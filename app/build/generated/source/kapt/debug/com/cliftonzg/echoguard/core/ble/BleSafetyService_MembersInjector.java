package com.cliftonzg.echoguard.core.ble;

import android.os.Vibrator;
import dagger.MembersInjector;
import dagger.internal.DaggerGenerated;
import dagger.internal.InjectedFieldSignature;
import dagger.internal.QualifierMetadata;
import javax.annotation.processing.Generated;
import javax.inject.Provider;

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
public final class BleSafetyService_MembersInjector implements MembersInjector<BleSafetyService> {
  private final Provider<BleScannerEngine> scannerEngineProvider;

  private final Provider<Vibrator> vibratorProvider;

  public BleSafetyService_MembersInjector(Provider<BleScannerEngine> scannerEngineProvider,
      Provider<Vibrator> vibratorProvider) {
    this.scannerEngineProvider = scannerEngineProvider;
    this.vibratorProvider = vibratorProvider;
  }

  public static MembersInjector<BleSafetyService> create(
      Provider<BleScannerEngine> scannerEngineProvider, Provider<Vibrator> vibratorProvider) {
    return new BleSafetyService_MembersInjector(scannerEngineProvider, vibratorProvider);
  }

  @Override
  public void injectMembers(BleSafetyService instance) {
    injectScannerEngine(instance, scannerEngineProvider.get());
    injectVibrator(instance, vibratorProvider.get());
  }

  @InjectedFieldSignature("com.cliftonzg.echoguard.core.ble.BleSafetyService.scannerEngine")
  public static void injectScannerEngine(BleSafetyService instance,
      BleScannerEngine scannerEngine) {
    instance.scannerEngine = scannerEngine;
  }

  @InjectedFieldSignature("com.cliftonzg.echoguard.core.ble.BleSafetyService.vibrator")
  public static void injectVibrator(BleSafetyService instance, Vibrator vibrator) {
    instance.vibrator = vibrator;
  }
}
