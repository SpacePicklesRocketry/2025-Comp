import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

TARGET_ALTITUDE = 790  # meters

def load_and_convert(filepath):
    """Load raw rocket flight CSV and return time (s) and altitude"""
    df = pd.read_csv(filepath)
    df['time'] = df['Time'] / 1000.0  # Convert ms to seconds
    return df[['time', 'Altitude']].rename(columns={'Altitude': 'altitude'}).dropna()

def interpolate(df, new_times):
    """Interpolate altitude over new time points"""
    return np.interp(new_times, df['time'], df['altitude'])

def simulate_airbrake_deploy(no_ab_df, full_ab_df):
    times = no_ab_df['time'].values
    no_ab_alt = no_ab_df['altitude'].values

    best_error = float('inf')
    best_switch_time = None
    best_profile = None

    for i in range(1, len(times)):
        switch_time = times[i]
        before_time = times[:i]
        after_time = times[i:] - switch_time

        alt_before = no_ab_alt[:i]
        alt_after_interp = interpolate(full_ab_df, after_time)
        alt_after = alt_before[-1] + alt_after_interp

        full_altitude = np.concatenate([alt_before, alt_after])
        final_alt = full_altitude[-1]
        error = abs(final_alt - TARGET_ALTITUDE)

        if error < best_error:
            best_error = error
            best_switch_time = switch_time
            best_profile = (times, full_altitude)

    return best_switch_time, best_profile, best_error

def plot_results(no_ab_df, full_ab_df, times, profile, best_time):
    plt.plot(no_ab_df['time'], no_ab_df['altitude'], label='No Airbrakes', linestyle='--')
    plt.plot(full_ab_df['time'], full_ab_df['altitude'], label='Full Airbrakes', linestyle='--')
    plt.plot(times, profile, label='Optimal Deployment', linewidth=2)

    plt.axhline(TARGET_ALTITUDE, color='gray', linestyle=':', label=f'Target Altitude ({TARGET_ALTITUDE}m)')
    plt.axvline(best_time, color='red', linestyle=':', label=f'Switch @ {best_time:.2f}s')

    plt.xlabel("Time (s)")
    plt.ylabel("Altitude (m)")
    plt.title("Airbrake Optimization to Hit Target Altitude")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

def main():
    print("🛠️  Airbrake Optimization Script\n")

    no_ab_path = input("Path to no-airbrake CSV: ").strip()
    full_ab_path = input("Path to full-airbrake CSV: ").strip()

    print("📦 Loading and converting CSVs...")
    no_ab_df = load_and_convert(no_ab_path)
    full_ab_df = load_and_convert(full_ab_path)

    print("🔍 Simulating airbrake deployments...")
    best_time, (times, profile), error = simulate_airbrake_deploy(no_ab_df, full_ab_df)

    print(f"\n✅ Best airbrake deployment time: {best_time:.2f} s")
    print(f"🎯 Final altitude: {profile[-1]:.2f} m (error = {error:.2f} m)")

    print("\n📊 Plotting results...")
    plot_results(no_ab_df, full_ab_df, times, profile, best_time)

if __name__ == "__main__":
    main()