type RobotSafetyStopCallback = (reason: string) => void;

let robotSafetyStopCallback: RobotSafetyStopCallback | undefined = undefined;

export function registerRobotSafetyStop(callback: RobotSafetyStopCallback): void {
  robotSafetyStopCallback = callback;
}

export function unregisterRobotSafetyStop(callback: RobotSafetyStopCallback): void {
  if (robotSafetyStopCallback === callback) {
    robotSafetyStopCallback = undefined;
  }
}

export function requestRobotSafetyStop(reason: string): void {
  if (robotSafetyStopCallback !== undefined) {
    robotSafetyStopCallback(reason);
  }
}
