/*
 * Local compile-time fallback for DevEco CLI builds that do not add the HMS
 * NearLink SDK declaration path. Runtime still imports the system NearLink Kit.
 */
declare module '@kit.NearLinkKit' {
  export namespace constant {
    export enum ConnectionState {
      STATE_DISCONNECTED = 0,
      STATE_CONNECTING = 1,
      STATE_CONNECTED = 2,
      STATE_DISCONNECTING = 3
    }
  }

  export namespace scan {
    export interface ScanResults {
      address: string;
      deviceName?: string;
      rssi?: number;
      data?: ArrayBuffer;
    }

    export interface ScanFilters {
      deviceName?: string;
      deviceId?: string;
      address?: string;
      serviceUuid?: string;
    }

    export interface ScanOptions {
      scanMode?: number;
      dutyRatio?: number;
      matchMode?: number;
    }

    export function startScan(filters?: Array<ScanFilters>, options?: ScanOptions): Promise<void>;
    export function stopScan(): Promise<void>;
    export function on(type: 'deviceFound', callback: (data: Array<ScanResults>) => void): void;
    export function off(type: 'deviceFound'): void;
  }

  export namespace advertising {
    export interface AdvertisingSettings {
      interval?: number;
      power?: number;
      txPower?: number;
      connectable?: boolean;
    }

    export interface ManufacturerData {
      manufacturerId?: number;
      manufacturerData?: ArrayBufferLike;
      manufactureId?: number;
      manufactureValue?: ArrayBufferLike;
    }

    export interface ServiceData {
      serviceUuid?: string;
      serviceData?: ArrayBufferLike;
      serviceValue?: ArrayBufferLike;
    }

    export interface AdvertisingData {
      serviceUuids?: Array<string>;
      manufacturerData?: Array<ManufacturerData>;
      manufactureData?: Array<ManufacturerData>;
      serviceData?: Array<ServiceData>;
      includeDeviceName?: boolean;
    }

    export interface AdvertisingParams {
      advertisingSettings?: AdvertisingSettings;
      advertisingData?: AdvertisingData;
      advertisingResponse?: AdvertisingData;
    }

    export function startAdvertising(params: AdvertisingParams): Promise<number>;
    export function stopAdvertising(handle: number): Promise<void>;
  }

  export namespace manager {
    export function getLocalName(): string;
  }

  export namespace ssap {
    export enum Operation {
      READABLE = 1,
      WRITE_NO_RESPONSE = 2,
      WRITE_WITH_RESPONSE = 4,
      NOTIFY = 8
    }

    export enum PropertyWriteType {
      WRITE = 0,
      WRITE_NO_RESPONSE = 1
    }

    export enum PropertyDescriptorType {
      CLIENT_PROPERTY_CONFIG = 1,
      PROPERTY = 2
    }

    export interface PropertyDescriptor {
      serviceUuid?: string;
      propertyUuid?: string;
      descriptorUuid?: string;
      descriptorType?: PropertyDescriptorType;
      value?: ArrayBuffer;
      isWriteable?: boolean;
    }

    export interface Property {
      serviceUuid: string;
      propertyUuid: string;
      value?: ArrayBuffer;
      operation?: number;
      descriptors?: Array<PropertyDescriptor>;
    }

    export interface Service {
      serviceUuid: string;
      isPrimary?: boolean;
      properties?: Array<Property>;
    }

    export interface ConnectionChangeState {
      address: string;
      state: number;
      reason?: number;
    }

    export interface PropertyReadRequest {
      address: string;
      transId?: number;
      serviceUuid?: string;
      propertyUuid?: string;
      property: Property;
      value?: ArrayBuffer;
    }

    export interface PropertyWriteRequest {
      address: string;
      transId?: number;
      serviceUuid?: string;
      propertyUuid?: string;
      property: Property;
      value?: ArrayBuffer;
    }

    export interface Client {
      connect(): Promise<void>;
      disconnect(): Promise<void>;
      close(): void;
      getServices(): Promise<Array<Service>>;
      readProperty(property: Property): Promise<Property>;
      writeProperty(property: Property, type?: PropertyWriteType): Promise<void>;
      setPropertyNotification(property: Property, enable: boolean): Promise<void>;
      on(type: 'connectionStateChange', callback: (data: ConnectionChangeState) => void): void;
      on(type: 'propertyChange', callback: (data: Property) => void): void;
      off(type: 'connectionStateChange' | 'propertyChange'): void;
    }

    export interface Server {
      addService(service: Service): Promise<void>;
      removeService(serviceUuid: string): Promise<void>;
      close(): void;
      notifyPropertyChanged(address: string, property: Property): Promise<void>;
      sendResponse?(address: string, transId: number, status: number, property: Property): Promise<void>;
      on(type: 'connectionStateChange', callback: (data: ConnectionChangeState) => void): void;
      on(type: 'propertyRead', callback: (data: PropertyReadRequest) => void): void;
      on(type: 'propertyWrite', callback: (data: PropertyWriteRequest) => void): void;
      off(type: 'connectionStateChange' | 'propertyRead' | 'propertyWrite'): void;
    }

    export function createClient(address: string): Client;
    export function createServer(): Server;
  }
}
