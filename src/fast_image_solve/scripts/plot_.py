import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def read_data_file(filename):
    """Read data file"""
    try:
        df = pd.read_csv(filename)
        return df
    except Exception as e:
        print(f"Failed to read file {filename}: {e}")
        return None

def filter_points_in_range(df, x_range=(0, 360), y_range=(0, 320)):
    """Filter points within specified range (based on file1 coordinates)"""
    filtered_df = df[
        (df['World_X'] >= x_range[0]) & (df['World_X'] <= x_range[1]) &
        (df['World_Y'] >= y_range[0]) & (df['World_Y'] <= y_range[1])
    ]
    return filtered_df

def calculate_differences_in_range(df1, df2, x_range=(0, 360), y_range=(0, 320)):
    """Calculate differences only for points that are within range in file1"""
    # Filter points in range based on file1 coordinates
    filtered_df1 = filter_points_in_range(df1, x_range, y_range)
    
    print(f"File 1 points in range: {len(filtered_df1)}")
    print(f"File 2 total points: {len(df2)}")
    
    # Merge dataframes based on ID (only points from file1 that are in range)
    merged = pd.merge(filtered_df1, df2, on=['ID'], suffixes=('_file1', '_file2'))
    
    if len(merged) == 0:
        print("No matching IDs found!")
        return None
    
    print(f"Matching points for comparison: {len(merged)}")
    
    # Calculate coordinate differences (no range restriction on differences)
    merged['diff_X'] = np.abs(merged['World_X_file1'] - merged['World_X_file2'])
    merged['diff_Y'] = np.abs(merged['World_Y_file1'] - merged['World_Y_file2'])
    merged['diff_Z'] = np.abs(merged['World_Z_file1'] - merged['World_Z_file2'])
    merged['diff_total'] = np.sqrt(merged['diff_X']**2 + merged['diff_Y']**2 + merged['diff_Z']**2)
    
    return merged

def plot_coordinate_differences(merged_df):
    """Plot coordinate differences"""
    if merged_df is None or len(merged_df) == 0:
        print("No data to plot")
        return
    
    # Sort by ID
    merged_df = merged_df.sort_values('ID')
    
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(15, 12))
    
    ids = merged_df['ID'].astype(int)
    x_pos = range(len(ids))
    
    # Top plot: X, Y, Z axis differences comparison
    width = 0.25
    bars1 = ax1.bar(np.array(x_pos) - width, merged_df['diff_X'], width, 
                    label='X-axis difference', alpha=0.8, color='red')
    bars2 = ax1.bar(np.array(x_pos), merged_df['diff_Y'], width, 
                    label='Y-axis difference', alpha=0.8, color='blue')
    bars3 = ax1.bar(np.array(x_pos) + width, merged_df['diff_Z'], width, 
                    label='Z-axis difference', alpha=0.8, color='green')
    
    ax1.set_xlabel('Point ID')
    ax1.set_ylabel('Coordinate difference (mm)')
    ax1.set_title('Coordinate Differences by Axis (Only points with File1 in range 0<X<360, 0<Y<320)')
    ax1.set_xticks(x_pos)
    ax1.set_xticklabels(ids, rotation=45, ha='right')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    # Add value labels on bars
    for i, (bar, diff) in enumerate(zip(bars1, merged_df['diff_X'])):
        if diff > 0:
            ax1.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.1,
                    f'{diff:.3f}', ha='center', va='bottom', fontsize=6, rotation=45)
    
    for i, (bar, diff) in enumerate(zip(bars2, merged_df['diff_Y'])):
        if diff > 0:
            ax1.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.1,
                    f'{diff:.3f}', ha='center', va='bottom', fontsize=6, rotation=45)
    
    for i, (bar, diff) in enumerate(zip(bars3, merged_df['diff_Z'])):
        if diff > 0:
            ax1.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.1,
                    f'{diff:.3f}', ha='center', va='bottom', fontsize=6, rotation=45)
    
    # Bottom plot: Total coordinate differences
    bars4 = ax2.bar(x_pos, merged_df['diff_total'], alpha=0.7, color='orange')
    ax2.set_xlabel('Point ID')
    ax2.set_ylabel('Total coordinate difference (mm)')
    ax2.set_title('Total Coordinate Differences')
    ax2.set_xticks(x_pos)
    ax2.set_xticklabels(ids, rotation=45, ha='right')
    ax2.grid(True, alpha=0.3)
    
    # Add value labels on bars
    for i, (bar, diff) in enumerate(zip(bars4, merged_df['diff_total'])):
        if diff > 0.001:
            ax2.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.1,
                    f'{diff:.3f}', ha='center', va='bottom', fontsize=7, rotation=45)
    
    plt.tight_layout()
    plt.suptitle('Coordinate Differences Analysis (Based on File1 range filtering)', fontsize=16, y=0.98)
    plt.show()

def plot_detailed_analysis(merged_df):
    """Plot detailed analysis"""
    if merged_df is None or len(merged_df) == 0:
        print("No data to plot")
        return
    
    merged_df = merged_df.sort_values('ID')
    
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 12))
    
    ids = merged_df['ID'].astype(int)
    x_pos = range(len(ids))
    
    # Top left: X-axis difference scatter plot
    scatter1 = ax1.scatter(x_pos, merged_df['diff_X'], c=merged_df['diff_X'], 
                          cmap='Reds', s=60, alpha=0.7)
    ax1.set_xlabel('Point index')
    ax1.set_ylabel('X-axis difference (mm)')
    ax1.set_title('X-axis Difference Distribution')
    ax1.grid(True, alpha=0.3)
    plt.colorbar(scatter1, ax=ax1)
    
    # Top right: Y-axis difference scatter plot
    scatter2 = ax2.scatter(x_pos, merged_df['diff_Y'], c=merged_df['diff_Y'], 
                          cmap='Blues', s=60, alpha=0.7)
    ax2.set_xlabel('Point index')
    ax2.set_ylabel('Y-axis difference (mm)')
    ax2.set_title('Y-axis Difference Distribution')
    ax2.grid(True, alpha=0.3)
    plt.colorbar(scatter2, ax=ax2)
    
    # Bottom left: Z-axis difference scatter plot
    scatter3 = ax3.scatter(x_pos, merged_df['diff_Z'], c=merged_df['diff_Z'], 
                          cmap='Greens', s=60, alpha=0.7)
    ax3.set_xlabel('Point index')
    ax3.set_ylabel('Z-axis difference (mm)')
    ax3.set_title('Z-axis Difference Distribution')
    ax3.grid(True, alpha=0.3)
    plt.colorbar(scatter3, ax=ax3)
    
    # Bottom right: Cumulative difference analysis
    sorted_diffs = np.sort(merged_df['diff_total'])
    y_vals = np.arange(1, len(sorted_diffs) + 1) / len(sorted_diffs)
    ax4.plot(sorted_diffs, y_vals, 'r-', linewidth=2)
    ax4.set_xlabel('Total coordinate difference (mm)')
    ax4.set_ylabel('Cumulative probability')
    ax4.set_title('Cumulative Distribution of Coordinate Differences')
    ax4.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.suptitle('Detailed Coordinate Difference Analysis', fontsize=16, y=0.98)
    plt.show()

def print_detailed_differences(merged_df):
    """Print detailed difference information"""
    if merged_df is None or len(merged_df) == 0:
        print("No data to display")
        return
    
    merged_df = merged_df.sort_values('ID')
    
    print("\n" + "="*130)
    print("Detailed coordinate difference analysis (Only points with File1 in range 0<X<360, 0<Y<320)")
    print("="*130)
    print(f"{'ID':<5} {'File1(X,Y,Z)':<30} {'File2(X,Y,Z)':<30} {'ΔX':<8} {'ΔY':<8} {'ΔZ':<8} {'Total':<10}")
    print("-"*130)
    
    for _, row in merged_df.iterrows():
        id_val = int(row['ID'])
        x1, y1, z1 = row['World_X_file1'], row['World_Y_file1'], row['World_Z_file1']
        x2, y2, z2 = row['World_X_file2'], row['World_Y_file2'], row['World_Z_file2']
        dx, dy, dz = row['diff_X'], row['diff_Y'], row['diff_Z']
        total_diff = row['diff_total']
        
        print(f"{id_val:<5} ({x1:>8.3f},{y1:>8.3f},{z1:>8.3f})  ({x2:>8.3f},{y2:>8.3f},{z2:>8.3f})  "
              f"{dx:<8.3f} {dy:<8.3f} {dz:<8.3f} {total_diff:<10.3f}")
    
    print("\n" + "="*130)
    print("Statistical summary:")
    print("="*130)
    print(f"Points compared: {len(merged_df)}")
    print(f"Average X difference: {merged_df['diff_X'].mean():.6f} mm")
    print(f"Average Y difference: {merged_df['diff_Y'].mean():.6f} mm")
    print(f"Average Z difference: {merged_df['diff_Z'].mean():.6f} mm")
    print(f"Average total difference: {merged_df['diff_total'].mean():.6f} mm")
    print(f"Maximum total difference: {merged_df['diff_total'].max():.6f} mm")
    print(f"Difference standard deviation: {merged_df['diff_total'].std():.6f} mm")

def analyze_range_statistics(df1, df2, x_range=(0, 360), y_range=(0, 320)):
    """Analyze range statistics for both files"""
    print("\n" + "="*60)
    print("Data range statistics")
    print("="*60)
    
    # File 1 statistics
    file1_in_range = filter_points_in_range(df1, x_range, y_range)
    print(f"File 1:")
    print(f"  Total points: {len(df1)}")
    print(f"  Points in range (0<X<360, 0<Y<320): {len(file1_in_range)}")
    print(f"  Points out of range: {len(df1) - len(file1_in_range)}")
    
    # File 2 statistics (for matching IDs)
    matching_ids = set(file1_in_range['ID'])
    file2_matching = df2[df2['ID'].isin(matching_ids)]
    print(f"File 2:")
    print(f"  Total points: {len(df2)}")
    print(f"  Points matching File1 filtered IDs: {len(file2_matching)}")

def highlight_large_differences(merged_df, threshold=0.1):
    """Highlight points with large differences"""
    if merged_df is None or len(merged_df) == 0:
        return
    
    large_diffs = merged_df[merged_df['diff_total'] > threshold].sort_values('diff_total', ascending=False)
    
    if len(large_diffs) > 0:
        print(f"\n" + "="*90)
        print(f"Points with differences > {threshold} mm (Total: {len(large_diffs)})")
        print("Only points with File1 coordinates in range (0<X<360, 0<Y<320)")
        print("="*90)
        print(f"{'ID':<5} {'Total_diff(mm)':<15} {'X_diff(mm)':<12} {'Y_diff(mm)':<12} {'Z_diff(mm)':<12}")
        print("-"*90)
        for _, row in large_diffs.iterrows():
            print(f"{int(row['ID']):<5} {row['diff_total']:<15.6f} "
                  f"{row['diff_X']:<12.6f} {row['diff_Y']:<12.6f} {row['diff_Z']:<12.6f}")

def main():
    # File paths
    file1 = "/home/hyq/icc/lashingrobots/src/fast_image_solve/data/FullRegioncoordinates.txt"  # Replace with your first file path
    file2 = "/home/hyq/icc/lashingrobots/src/fast_image_solve/data/NineRegioncoordinates.txt"  # Replace with your second file path
    
    print("Reading data files...")
    df1 = read_data_file(file1)
    df2 = read_data_file(file2)
    
    if df1 is None or df2 is None:
        print("Failed to read files, please check file paths and format!")
        return
    
    print(f"File 1 contains {len(df1)} rows")
    print(f"File 2 contains {len(df2)} rows")
    
    # Analyze range statistics
    analyze_range_statistics(df1, df2, x_range=(0, 360), y_range=(0, 320))
    
    # Calculate differences (only for points with file1 in range)
    print("\nCalculating coordinate differences (only for points with File1 in range)...")
    merged_df = calculate_differences_in_range(df1, df2, x_range=(0, 360), y_range=(0, 320))
    
    # Print detailed differences
    print_detailed_differences(merged_df)
    
    # Highlight large differences
    highlight_large_differences(merged_df, threshold=0.05)
    
    # Generate plots
    if merged_df is not None and len(merged_df) > 0:
        print("Generating plots...")
        plot_coordinate_differences(merged_df)
        plot_detailed_analysis(merged_df)
    
    print("Analysis completed!")

if __name__ == "__main__":
    main()