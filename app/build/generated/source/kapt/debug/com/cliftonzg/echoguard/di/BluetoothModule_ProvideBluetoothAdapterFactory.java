package com.cliftonzg.echoguard.di;

import android.bluetooth.BluetoothAdapter;
import android.content.Context;
import dagger.internal.DaggerGenerated;
import dagger.internal.Factory;
import dagger.internal.QualifierMetadata;
import dagger.internal.ScopeMetadata;
import javax.annotation.processing.Generated;
import javax.inject.Provider;
import org.jetbrains.annotations.Nullable;

@ScopeMetadata("javax.inject.Singleton")
@QualifierMetadata("dagger.hilt.android.qualifiers.ApplicationContext")
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
public final class BluetoothModule_ProvideBluetoothAdapterFactory implements Factory<BluetoothAdapter> {
  private final Provider<Context> contextProvider;

  public BluetoothModule_ProvideBluetoothAdapterFactory(Provider<Context> contextProvider) {
    this.contextProvider = contextProvider;
  }

  @Override
  @Nullable
  public BluetoothAdapter get() {
    return provideBluetoothAdapter(contextProvider.get());
  }

  public static BluetoothModule_ProvideBluetoothAdapterFactory create(
      Provider<Context> contextProvider) {
    return new BluetoothModule_ProvideBluetoothAdapterFactory(contextProvider);
  }

  @Nullable
  public static BluetoothAdapter provideBluetoothAdapter(Context context) {
    return BluetoothModule.INSTANCE.provideBluetoothAdapter(context);
  }
}
