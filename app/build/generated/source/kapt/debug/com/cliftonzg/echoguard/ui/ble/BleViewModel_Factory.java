package com.cliftonzg.echoguard.ui.ble;

import com.cliftonzg.echoguard.core.ble.BleScannerEngine;
import dagger.internal.DaggerGenerated;
import dagger.internal.Factory;
import dagger.internal.QualifierMetadata;
import dagger.internal.ScopeMetadata;
import javax.annotation.processing.Generated;
import javax.inject.Provider;

@ScopeMetadata
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
public final class BleViewModel_Factory implements Factory<BleViewModel> {
  private final Provider<BleScannerEngine> scannerEngineProvider;

  public BleViewModel_Factory(Provider<BleScannerEngine> scannerEngineProvider) {
    this.scannerEngineProvider = scannerEngineProvider;
  }

  @Override
  public BleViewModel get() {
    return newInstance(scannerEngineProvider.get());
  }

  public static BleViewModel_Factory create(Provider<BleScannerEngine> scannerEngineProvider) {
    return new BleViewModel_Factory(scannerEngineProvider);
  }

  public static BleViewModel newInstance(BleScannerEngine scannerEngine) {
    return new BleViewModel(scannerEngine);
  }
}
