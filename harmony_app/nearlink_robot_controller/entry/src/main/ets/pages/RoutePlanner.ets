import { image } from '@kit.ImageKit';

export interface RoutePointModel {
  x: number;
  y: number;
}

export interface RouteLegModel {
  forwardMs: number;
  turnDirection: number;
  turnMs: number;
  turnDegrees: number;
}

export interface RouteCalibrationModel {
  secondsPerMeterAtCruiseSpeed: number;
  ninetyDegreeTurnMsAtCruiseSpeed: number;
}

export interface RoutePlanModel {
  points: Array<RoutePointModel>;
  legs: Array<RouteLegModel>;
  totalMeters: number;
  drawingLength: number;
  estimatedSeconds: number;
  closed: boolean;
}

export interface RouteImageRecognitionModel {
  points: Array<RoutePointModel>;
  closed: boolean;
  hasStartMarker: boolean;
  hasEndMarker: boolean;
}

interface RouteBfsResult {
  farthest: number;
  parents: Int32Array;
}

interface RouteImageBounds {
  minX: number;
  minY: number;
  maxX: number;
  maxY: number;
}

const ROUTE_MAX_SEGMENTS: number = 12;
const ROUTE_CRUISE_SPEED_SECONDS_PER_METER: number = 2.8;
const ROUTE_CRUISE_SPEED_NINETY_DEGREE_TURN_MS: number = 686;

export class RoutePlanner {
  static defaultSquare(): Array<RoutePointModel> {
    return [
      { x: 0.14, y: 0.20 },
      { x: 0.86, y: 0.20 },
      { x: 0.86, y: 0.80 },
      { x: 0.14, y: 0.80 },
      { x: 0.14, y: 0.20 }
    ];
  }

  static pointDistance(a: RoutePointModel, b: RoutePointModel): number {
    let dx = a.x - b.x;
    let dy = a.y - b.y;
    return Math.sqrt(dx * dx + dy * dy);
  }

  static simplify(points: Array<RoutePointModel>, tolerance: number = 0.022): Array<RoutePointModel> {
    let cleaned = new Array<RoutePointModel>();
    for (let index = 0; index < points.length; index++) {
      let point = points[index];
      if (cleaned.length === 0 || RoutePlanner.pointDistance(cleaned[cleaned.length - 1], point) >= 0.008) {
        cleaned.push({ x: RoutePlanner.clamp(point.x, 0, 1), y: RoutePlanner.clamp(point.y, 0, 1) });
      }
    }
    if (cleaned.length <= 2) {
      return cleaned;
    }

    let closed = RoutePlanner.pointDistance(cleaned[0], cleaned[cleaned.length - 1]) <= 0.06;
    if (closed) {
      cleaned[cleaned.length - 1] = { x: cleaned[0].x, y: cleaned[0].y };
    }

    let simplified: Array<RoutePointModel>;
    if (closed && cleaned.length >= 6) {
      let middle = Math.floor((cleaned.length - 1) / 2);
      let firstHalf = RoutePlanner.rdp(cleaned.slice(0, middle + 1), tolerance);
      let secondHalf = RoutePlanner.rdp(cleaned.slice(middle), tolerance);
      simplified = firstHalf.slice(0, firstHalf.length - 1).concat(secondHalf);
      if (RoutePlanner.pointDistance(simplified[0], simplified[simplified.length - 1]) > 0.001) {
        simplified.push({ x: simplified[0].x, y: simplified[0].y });
      }
    } else {
      simplified = RoutePlanner.rdp(cleaned, tolerance);
    }

    return RoutePlanner.limitPoints(simplified, ROUTE_MAX_SEGMENTS + 1, closed);
  }

  static buildPlan(points: Array<RoutePointModel>, totalMeters: number, speed: number,
    calibration?: RouteCalibrationModel): RoutePlanModel {
    let plannedPoints = RoutePlanner.simplify(points);
    let legs = new Array<RouteLegModel>();
    let closed = plannedPoints.length >= 4 &&
      RoutePlanner.pointDistance(plannedPoints[0], plannedPoints[plannedPoints.length - 1]) <= 0.01;
    if (plannedPoints.length < 2) {
      return {
        points: plannedPoints, legs: legs, totalMeters: totalMeters, drawingLength: 0,
        estimatedSeconds: 0, closed: false
      };
    }

    let segmentCount = Math.min(plannedPoints.length - 1, ROUTE_MAX_SEGMENTS);
    let lengths = new Array<number>();
    let totalDrawingLength = 0;
    for (let index = 0; index < segmentCount; index++) {
      let length = RoutePlanner.pointDistance(plannedPoints[index], plannedPoints[index + 1]);
      lengths.push(length);
      totalDrawingLength += length;
    }
    if (totalDrawingLength <= 0.001) {
      return {
        points: plannedPoints, legs: legs, totalMeters: totalMeters, drawingLength: 0,
        estimatedSeconds: 0, closed: closed
      };
    }

    let safeMeters = RoutePlanner.clamp(totalMeters, 0.5, 200);
    let safeSpeed = Math.round(RoutePlanner.clamp(speed, 30, 100));
    let secondsPerMeter = RoutePlanner.clamp(
      calibration === undefined ? ROUTE_CRUISE_SPEED_SECONDS_PER_METER : calibration.secondsPerMeterAtCruiseSpeed,
      0.15, 5);
    let ninetyDegreeTurnMs = RoutePlanner.clamp(
      calibration === undefined ? ROUTE_CRUISE_SPEED_NINETY_DEGREE_TURN_MS : calibration.ninetyDegreeTurnMsAtCruiseSpeed,
      200, 4000);
    let speedCompensation = 100 / safeSpeed;
    let estimatedSeconds = 0;
    for (let index = 0; index < segmentCount; index++) {
      let segmentMeters = safeMeters * lengths[index] / totalDrawingLength;
      let forwardMs = Math.round(segmentMeters * secondsPerMeter * 1000 * speedCompensation);
      forwardMs = Math.round(RoutePlanner.clamp(forwardMs, 500, 60000));
      let turnDirection = 0;
      let turnMs = 0;
      let turnDegrees = 0;
      let hasNext = index + 1 < segmentCount;
      if (!hasNext && closed) {
        hasNext = true;
      }
      if (hasNext) {
        let nextIndex = index + 1 < segmentCount ? index + 1 : 0;
        let currentStart = plannedPoints[index];
        let currentEnd = plannedPoints[index + 1];
        let nextEnd = plannedPoints[nextIndex + 1];
        let ax = currentEnd.x - currentStart.x;
        let ay = currentEnd.y - currentStart.y;
        let bx = nextEnd.x - plannedPoints[nextIndex].x;
        let by = nextEnd.y - plannedPoints[nextIndex].y;
        let aLength = Math.sqrt(ax * ax + ay * ay);
        let bLength = Math.sqrt(bx * bx + by * by);
        if (aLength > 0.001 && bLength > 0.001) {
          let cosine = RoutePlanner.clamp((ax * bx + ay * by) / (aLength * bLength), -1, 1);
          let angle = Math.acos(cosine) * 180 / Math.PI;
          if (angle >= 12) {
            let cross = ax * by - ay * bx;
            turnDirection = cross >= 0 ? 2 : 1;
            turnDegrees = Math.round(angle);
            turnMs = Math.round(RoutePlanner.clamp(
              angle / 90 * ninetyDegreeTurnMs * speedCompensation, 100, 8000));
          }
        }
      }
      legs.push({ forwardMs: forwardMs, turnDirection: turnDirection, turnMs: turnMs, turnDegrees: turnDegrees });
      estimatedSeconds += (forwardMs + turnMs) / 1000;
    }

    return {
      points: plannedPoints,
      legs: legs,
      totalMeters: safeMeters,
      drawingLength: totalDrawingLength,
      estimatedSeconds: estimatedSeconds,
      closed: closed
    };
  }

  static rdp(points: Array<RoutePointModel>, epsilon: number): Array<RoutePointModel> {
    if (points.length <= 2) {
      return points.slice();
    }
    let maxDistance = 0;
    let maxIndex = 0;
    for (let index = 1; index < points.length - 1; index++) {
      let distance = RoutePlanner.pointLineDistance(points[index], points[0], points[points.length - 1]);
      if (distance > maxDistance) {
        maxDistance = distance;
        maxIndex = index;
      }
    }
    if (maxDistance <= epsilon) {
      return [points[0], points[points.length - 1]];
    }
    let left = RoutePlanner.rdp(points.slice(0, maxIndex + 1), epsilon);
    let right = RoutePlanner.rdp(points.slice(maxIndex), epsilon);
    return left.slice(0, left.length - 1).concat(right);
  }

  static pointLineDistance(point: RoutePointModel, start: RoutePointModel, end: RoutePointModel): number {
    let dx = end.x - start.x;
    let dy = end.y - start.y;
    let lengthSquared = dx * dx + dy * dy;
    if (lengthSquared <= 0.000001) {
      return RoutePlanner.pointDistance(point, start);
    }
    let t = ((point.x - start.x) * dx + (point.y - start.y) * dy) / lengthSquared;
    t = RoutePlanner.clamp(t, 0, 1);
    let projection: RoutePointModel = { x: start.x + t * dx, y: start.y + t * dy };
    return RoutePlanner.pointDistance(point, projection);
  }

  static limitPoints(points: Array<RoutePointModel>, maxPoints: number, closed: boolean): Array<RoutePointModel> {
    if (points.length <= maxPoints) {
      return points;
    }
    let limited = new Array<RoutePointModel>();
    let sourceCount = closed ? points.length - 1 : points.length;
    let targetCount = closed ? maxPoints - 1 : maxPoints;
    for (let index = 0; index < targetCount; index++) {
      let sourceIndex = Math.round(index * (sourceCount - 1) / Math.max(1, targetCount - 1));
      limited.push(points[sourceIndex]);
    }
    if (closed) {
      limited.push({ x: limited[0].x, y: limited[0].y });
    }
    return limited;
  }

  static clamp(value: number, min: number, max: number): number {
    return Math.max(min, Math.min(max, value));
  }

  static reverseRoute(points: Array<RoutePointModel>): Array<RoutePointModel> {
    if (points.length < 2) {
      return points.slice();
    }
    let closed = RoutePlanner.pointDistance(points[0], points[points.length - 1]) <= 0.01;
    if (!closed) {
      return points.slice().reverse();
    }
    let ring = points.slice(0, points.length - 1).reverse();
    ring.push({ x: ring[0].x, y: ring[0].y });
    return ring;
  }

  static rotateClosedRouteToPoint(points: Array<RoutePointModel>, target: RoutePointModel): Array<RoutePointModel> {
    if (points.length < 4 || RoutePlanner.pointDistance(points[0], points[points.length - 1]) > 0.01) {
      return points.slice();
    }
    let ring = points.slice(0, points.length - 1);
    let nearestIndex = 0;
    let nearestDistance = Number.MAX_VALUE;
    for (let index = 0; index < ring.length; index++) {
      let distance = RoutePlanner.pointDistance(ring[index], target);
      if (distance < nearestDistance) {
        nearestDistance = distance;
        nearestIndex = index;
      }
    }
    let rotated = ring.slice(nearestIndex).concat(ring.slice(0, nearestIndex));
    rotated.push({ x: rotated[0].x, y: rotated[0].y });
    return rotated;
  }
}

export class RouteImageRecognizer {
  static async getAspectRatio(uri: string): Promise<number> {
    let source = image.createImageSource(uri);
    try {
      let info = await source.getImageInfo();
      let width = info.size.width;
      let height = info.size.height;
      if (width <= 0 || height <= 0) {
        return 4 / 3;
      }
      return RoutePlanner.clamp(width / height, 0.5, 2.5);
    } finally {
      await source.release();
    }
  }

  static async recognize(uri: string): Promise<RouteImageRecognitionModel> {
    const targetWidth = 192;
    const targetHeight = 192;
    let source = image.createImageSource(uri);
    let pixelMap: image.PixelMap | undefined = undefined;
    try {
      pixelMap = await source.createPixelMap({
        desiredSize: { width: targetWidth, height: targetHeight },
        desiredPixelFormat: image.PixelMapFormat.RGBA_8888
      });
      let buffer = new ArrayBuffer(targetWidth * targetHeight * 4);
      await pixelMap.readPixelsToBuffer(buffer);
      let pixels = new Uint8Array(buffer);
      let startMarker = RouteImageRecognizer.findMarkerCenter(pixels, targetWidth, targetHeight, true);
      let endMarker = RouteImageRecognizer.findMarkerCenter(pixels, targetWidth, targetHeight, false);
      let mask = RouteImageRecognizer.makeRouteMask(pixels, targetWidth, targetHeight);
      mask = RouteImageRecognizer.keepLargestComponent(mask, targetWidth, targetHeight);
      RouteImageRecognizer.thin(mask, targetWidth, targetHeight);
      let endpoints = RouteImageRecognizer.findEndpoints(mask, targetWidth, targetHeight);
      let closed = endpoints.length === 0;
      let path = endpoints.length === 0 ? RouteImageRecognizer.traceClosedRoute(mask, targetWidth, targetHeight) :
        RouteImageRecognizer.traceOpenRoute(mask, targetWidth, targetHeight, endpoints[0]);
      if (path.length < 10) {
        throw new Error('No continuous route line was detected');
      }
      let points = RouteImageRecognizer.pathToPoints(path, targetWidth, targetHeight, closed);
      let bounds = RouteImageRecognizer.pathBounds(path, targetWidth);
      if (closed) {
        if (startMarker === undefined) {
          throw new Error('Closed routes require a green start marker');
        }
        points = RoutePlanner.rotateClosedRouteToPoint(
          points, RouteImageRecognizer.normalizePixelPoint(startMarker, bounds));
      } else {
        if (startMarker === undefined || endMarker === undefined) {
          throw new Error('Open routes require a green start marker and a red end marker');
        }
        let normalizedStart = RouteImageRecognizer.normalizePixelPoint(startMarker, bounds);
        let normalizedEnd = RouteImageRecognizer.normalizePixelPoint(endMarker, bounds);
        let firstDistance = RoutePlanner.pointDistance(points[0], normalizedStart);
        let lastDistance = RoutePlanner.pointDistance(points[points.length - 1], normalizedStart);
        if (lastDistance < firstDistance) {
          points = RoutePlanner.reverseRoute(points);
        }
        if (RoutePlanner.pointDistance(points[0], normalizedStart) > 0.18 ||
          RoutePlanner.pointDistance(points[points.length - 1], normalizedEnd) > 0.18) {
          throw new Error('Start or end marker is too far from the extracted route line');
        }
      }
      return {
        points: points,
        closed: closed,
        hasStartMarker: startMarker !== undefined,
        hasEndMarker: endMarker !== undefined
      };
    } finally {
      if (pixelMap !== undefined) {
        await pixelMap.release();
      }
      await source.release();
    }
  }

  static makeRouteMask(pixels: Uint8Array, width: number, height: number): Uint8Array {
    let luminance = new Uint8Array(width * height);
    let total = 0;
    for (let index = 0; index < width * height; index++) {
      let offset = index * 4;
      let value = Math.round(pixels[offset] * 0.299 + pixels[offset + 1] * 0.587 + pixels[offset + 2] * 0.114);
      luminance[index] = value;
      total += value;
    }
    let mean = total / (width * height);
    let threshold = Math.round(Math.max(55, Math.min(210, mean - 28)));
    let mask = new Uint8Array(width * height);
    for (let y = 2; y < height - 2; y++) {
      for (let x = 2; x < width - 2; x++) {
        let index = y * width + x;
        let offset = index * 4;
        mask[index] = luminance[index] < threshold &&
          !RouteImageRecognizer.isColorMarker(pixels[offset], pixels[offset + 1], pixels[offset + 2]) ? 1 : 0;
      }
    }
    return mask;
  }

  static isColorMarker(red: number, green: number, blue: number): boolean {
    return (green >= 100 && green > red * 1.25 && green > blue * 1.15) ||
      (red >= 130 && red > green * 1.45 && red > blue * 1.25);
  }

  static findMarkerCenter(pixels: Uint8Array, width: number, height: number, start: boolean): RoutePointModel | undefined {
    let totalX = 0;
    let totalY = 0;
    let count = 0;
    for (let y = 0; y < height; y++) {
      for (let x = 0; x < width; x++) {
        let offset = (y * width + x) * 4;
        let red = pixels[offset];
        let green = pixels[offset + 1];
        let blue = pixels[offset + 2];
        let matches = start ?
          green >= 100 && green > red * 1.25 && green > blue * 1.15 :
          red >= 130 && red > green * 1.45 && red > blue * 1.25;
        if (matches) {
          totalX += x;
          totalY += y;
          count++;
        }
      }
    }
    if (count < 5) {
      return undefined;
    }
    return { x: totalX / count, y: totalY / count };
  }

  static pathBounds(path: Array<number>, width: number): RouteImageBounds {
    let bounds: RouteImageBounds = { minX: width, minY: width, maxX: 0, maxY: 0 };
    for (let index = 0; index < path.length; index++) {
      let x = path[index] % width;
      let y = Math.floor(path[index] / width);
      bounds.minX = Math.min(bounds.minX, x);
      bounds.minY = Math.min(bounds.minY, y);
      bounds.maxX = Math.max(bounds.maxX, x);
      bounds.maxY = Math.max(bounds.maxY, y);
    }
    return bounds;
  }

  static normalizePixelPoint(point: RoutePointModel, bounds: RouteImageBounds): RoutePointModel {
    let spanX = Math.max(1, bounds.maxX - bounds.minX);
    let spanY = Math.max(1, bounds.maxY - bounds.minY);
    return {
      x: 0.08 + 0.84 * (point.x - bounds.minX) / spanX,
      y: 0.08 + 0.84 * (point.y - bounds.minY) / spanY
    };
  }

  static keepLargestComponent(mask: Uint8Array, width: number, height: number): Uint8Array {
    let visited = new Uint8Array(mask.length);
    let largest = new Array<number>();
    for (let index = 0; index < mask.length; index++) {
      if (mask[index] === 0 || visited[index] !== 0) {
        continue;
      }
      let component = new Array<number>();
      let queue = new Array<number>();
      queue.push(index);
      visited[index] = 1;
      let head = 0;
      while (head < queue.length) {
        let current = queue[head++];
        component.push(current);
        let neighbors = RouteImageRecognizer.neighbors(current, mask, width, height);
        for (let nextIndex = 0; nextIndex < neighbors.length; nextIndex++) {
          let next = neighbors[nextIndex];
          if (visited[next] === 0) {
            visited[next] = 1;
            queue.push(next);
          }
        }
      }
      if (component.length > largest.length) {
        largest = component;
      }
    }
    if (largest.length < 20) {
      return new Uint8Array(mask.length);
    }
    let output = new Uint8Array(mask.length);
    for (let index = 0; index < largest.length; index++) {
      output[largest[index]] = 1;
    }
    return output;
  }

  static thin(mask: Uint8Array, width: number, height: number): void {
    let changed = true;
    let iteration = 0;
    while (changed && iteration < 40) {
      changed = false;
      for (let pass = 0; pass < 2; pass++) {
        let remove = new Array<number>();
        for (let y = 1; y < height - 1; y++) {
          for (let x = 1; x < width - 1; x++) {
            let index = y * width + x;
            if (mask[index] === 0) {
              continue;
            }
            let p2 = mask[index - width];
            let p3 = mask[index - width + 1];
            let p4 = mask[index + 1];
            let p5 = mask[index + width + 1];
            let p6 = mask[index + width];
            let p7 = mask[index + width - 1];
            let p8 = mask[index - 1];
            let p9 = mask[index - width - 1];
            let count = p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9;
            let transitions = RouteImageRecognizer.transitionCount([p2, p3, p4, p5, p6, p7, p8, p9]);
            let firstGate = pass === 0 ? p2 * p4 * p6 === 0 && p4 * p6 * p8 === 0 :
              p2 * p4 * p8 === 0 && p2 * p6 * p8 === 0;
            if (count >= 2 && count <= 6 && transitions === 1 && firstGate) {
              remove.push(index);
            }
          }
        }
        if (remove.length > 0) {
          changed = true;
          for (let index = 0; index < remove.length; index++) {
            mask[remove[index]] = 0;
          }
        }
      }
      iteration++;
    }
  }

  static transitionCount(values: Array<number>): number {
    let transitions = 0;
    for (let index = 0; index < values.length; index++) {
      if (values[index] === 0 && values[(index + 1) % values.length] === 1) {
        transitions++;
      }
    }
    return transitions;
  }

  static findEndpoints(mask: Uint8Array, width: number, height: number): Array<number> {
    let endpoints = new Array<number>();
    for (let index = 0; index < mask.length; index++) {
      if (mask[index] !== 0 && RouteImageRecognizer.neighbors(index, mask, width, height).length === 1) {
        endpoints.push(index);
      }
    }
    return endpoints;
  }

  static traceOpenRoute(mask: Uint8Array, width: number, height: number, start: number): Array<number> {
    let first = RouteImageRecognizer.bfs(mask, width, height, start);
    let second = RouteImageRecognizer.bfs(mask, width, height, first.farthest);
    let path = new Array<number>();
    let cursor = second.farthest;
    while (cursor >= 0) {
      path.push(cursor);
      if (cursor === first.farthest) {
        break;
      }
      cursor = second.parents[cursor];
    }
    path.reverse();
    return path;
  }

  static traceClosedRoute(mask: Uint8Array, width: number, height: number): Array<number> {
    let start = -1;
    for (let index = 0; index < mask.length; index++) {
      if (mask[index] !== 0) {
        start = index;
        break;
      }
    }
    if (start < 0) {
      return new Array<number>();
    }
    let visited = new Uint8Array(mask.length);
    let path = new Array<number>();
    let previous = -1;
    let current = start;
    while (path.length <= mask.length) {
      path.push(current);
      visited[current] = 1;
      let candidates = RouteImageRecognizer.neighbors(current, mask, width, height);
      let next = -1;
      let bestScore = -1000;
      for (let index = 0; index < candidates.length; index++) {
        let candidate = candidates[index];
        if (candidate === previous || visited[candidate] !== 0) {
          continue;
        }
        let score = 0;
        if (previous >= 0) {
          let ax = current % width - previous % width;
          let ay = Math.floor(current / width) - Math.floor(previous / width);
          let bx = candidate % width - current % width;
          let by = Math.floor(candidate / width) - Math.floor(current / width);
          score = ax * bx + ay * by;
        }
        if (score > bestScore) {
          bestScore = score;
          next = candidate;
        }
      }
      if (next < 0) {
        if (path.length > 10 && RouteImageRecognizer.areNeighbors(current, start, width)) {
          path.push(start);
        }
        break;
      }
      previous = current;
      current = next;
    }
    if (path.length < 10 || path[path.length - 1] !== start) {
      return RouteImageRecognizer.traceOpenRoute(mask, width, height, start);
    }
    return path;
  }

  static bfs(mask: Uint8Array, width: number, height: number, start: number): RouteBfsResult {
    let parents = new Int32Array(mask.length);
    let distance = new Int32Array(mask.length);
    parents.fill(-1);
    distance.fill(-1);
    let queue = new Int32Array(mask.length);
    let head = 0;
    let tail = 0;
    queue[tail++] = start;
    distance[start] = 0;
    let farthest = start;
    while (head < tail) {
      let current = queue[head++];
      if (distance[current] > distance[farthest]) {
        farthest = current;
      }
      let neighbors = RouteImageRecognizer.neighbors(current, mask, width, height);
      for (let index = 0; index < neighbors.length; index++) {
        let next = neighbors[index];
        if (distance[next] < 0) {
          distance[next] = distance[current] + 1;
          parents[next] = current;
          queue[tail++] = next;
        }
      }
    }
    return { farthest: farthest, parents: parents };
  }

  static neighbors(index: number, mask: Uint8Array, width: number, height: number): Array<number> {
    let result = new Array<number>();
    let x = index % width;
    let y = Math.floor(index / width);
    for (let dy = -1; dy <= 1; dy++) {
      for (let dx = -1; dx <= 1; dx++) {
        if ((dx === 0 && dy === 0) || x + dx < 0 || x + dx >= width || y + dy < 0 || y + dy >= height) {
          continue;
        }
        let next = (y + dy) * width + x + dx;
        if (mask[next] !== 0) {
          result.push(next);
        }
      }
    }
    return result;
  }

  static areNeighbors(first: number, second: number, width: number): boolean {
    let dx = Math.abs(first % width - second % width);
    let dy = Math.abs(Math.floor(first / width) - Math.floor(second / width));
    return dx <= 1 && dy <= 1;
  }

  static pathToPoints(path: Array<number>, width: number, height: number, closed: boolean): Array<RoutePointModel> {
    let minX = width;
    let minY = height;
    let maxX = 0;
    let maxY = 0;
    for (let index = 0; index < path.length; index++) {
      let x = path[index] % width;
      let y = Math.floor(path[index] / width);
      minX = Math.min(minX, x);
      minY = Math.min(minY, y);
      maxX = Math.max(maxX, x);
      maxY = Math.max(maxY, y);
    }
    let spanX = Math.max(1, maxX - minX);
    let spanY = Math.max(1, maxY - minY);
    let sampleStep = Math.max(1, Math.floor(path.length / 160));
    let points = new Array<RoutePointModel>();
    for (let index = 0; index < path.length; index += sampleStep) {
      let x = path[index] % width;
      let y = Math.floor(path[index] / width);
      points.push({ x: 0.08 + 0.84 * (x - minX) / spanX, y: 0.08 + 0.84 * (y - minY) / spanY });
    }
    if (closed) {
      points.push({ x: points[0].x, y: points[0].y });
    } else {
      let lastIndex = path.length - 1;
      let lastX = path[lastIndex] % width;
      let lastY = Math.floor(path[lastIndex] / width);
      points.push({ x: 0.08 + 0.84 * (lastX - minX) / spanX, y: 0.08 + 0.84 * (lastY - minY) / spanY });
    }
    return RoutePlanner.simplify(points, 0.028);
  }
}
