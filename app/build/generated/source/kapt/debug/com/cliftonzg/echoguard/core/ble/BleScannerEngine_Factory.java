package com.cliftonzg.echoguard.core.ble;

import android.bluetooth.BluetoothAdapter;
import dagger.internal.DaggerGenerated;
import dagger.internal.Factory;
import dagger.internal.QualifierMetadata;
import dagger.internal.ScopeMetadata;
import javax.annotation.processing.Generated;
import javax.inject.Provider;

@ScopeMetadata("javax.inject.Singleton")
@QualifierMetadata
@DaggerGenerated
@Generated(
    value = "dagger.internal.codegen.ComponentProcessor",
    comments = "https://dagger.dev"
)
@SuppressWarnings({
    "unchecked",
    "rawtypes",
    "KotlinInternal",
    "KotlinInternalInJava"
})
public final class BleScannerEngine_Factory implements Factory<BleScannerEngine> {
  private final Provider<BluetoothAdapter> bluetoothAdapterProvider;

  public BleScannerEngine_Factory(Provider<BluetoothAdapter> bluetoothAdapterProvider) {
    this.bluetoothAdapterProvider = bluetoothAdapterProvider;
  }

  @Override
  public BleScannerEngine get() {
    return newInstance(bluetoothAdapterProvider.get());
  }

  public static BleScannerEngine_Factory create(
      Provider<BluetoothAdapter> bluetoothAdapterProvider) {
    return new BleScannerEngine_Factory(bluetoothAdapterProvider);
  }

  public static BleScannerEngine newInstance(BluetoothAdapter bluetoothAdapter) {
    return new BleScannerEngine(bluetoothAdapter);
  }
}
