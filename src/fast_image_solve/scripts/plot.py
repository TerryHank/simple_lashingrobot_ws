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

def calculate_pixel_differences(df1, df2):
    """Calculate pixel coordinate differences for same IDs"""
    # Merge dataframes based on ID
    merged = pd.merge(df1, df2, on=['ID'], suffixes=('_file1', '_file2'))
    
    # Calculate pixel coordinate differences
    merged['pixel_diff_X'] = np.abs(merged['Pixel_X_file1'] - merged['Pixel_X_file2'])
    merged['pixel_diff_Y'] = np.abs(merged['Pixel_Y_file1'] - merged['Pixel_Y_file2'])
    merged['pixel_diff_total'] = np.sqrt(merged['pixel_diff_X']**2 + merged['pixel_diff_Y']**2)
    
    return merged

def plot_pixel_differences_by_id(merged_df):
    """Plot pixel coordinate differences by ID"""
    # Sort by ID
    merged_df = merged_df.sort_values('ID')
    
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(15, 12))
    
    ids = merged_df['ID'].astype(int)
    x_pos = range(len(ids))
    
    # Top plot: X and Y axis pixel differences comparison
    width = 0.35
    bars1 = ax1.bar(np.array(x_pos) - width/2, merged_df['pixel_diff_X'], width, 
                    label='X-axis pixel difference', alpha=0.8, color='blue')
    bars2 = ax1.bar(np.array(x_pos) + width/2, merged_df['pixel_diff_Y'], width, 
                    label='Y-axis pixel difference', alpha=0.8, color='green')
    
    ax1.set_xlabel('Point ID')
    ax1.set_ylabel('Pixel difference')
    ax1.set_title('Pixel coordinate differences by point')
    ax1.set_xticks(x_pos)
    ax1.set_xticklabels(ids, rotation=45, ha='right')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    # Add value labels on bars
    for i, (bar, diff) in enumerate(zip(bars1, merged_df['pixel_diff_X'])):
        if diff > 0:
            ax1.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.1,
                    f'{diff:.1f}', ha='center', va='bottom', fontsize=6, rotation=45)
    
    for i, (bar, diff) in enumerate(zip(bars2, merged_df['pixel_diff_Y'])):
        if diff > 0:
            ax2.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.1,
                    f'{diff:.1f}', ha='center', va='bottom', fontsize=6, rotation=45)
    
    # Bottom plot: Total pixel differences
    bars3 = ax2.bar(x_pos, merged_df['pixel_diff_total'], alpha=0.7, color='orange')
    ax2.set_xlabel('Point ID')
    ax2.set_ylabel('Total pixel difference')
    ax2.set_title('Total pixel coordinate differences')
    ax2.set_xticks(x_pos)
    ax2.set_xticklabels(ids, rotation=45, ha='right')
    ax2.grid(True, alpha=0.3)
    
    # Add value labels on bars
    for i, (bar, diff) in enumerate(zip(bars3, merged_df['pixel_diff_total'])):
        if diff > 0.1:
            ax2.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.1,
                    f'{diff:.2f}', ha='center', va='bottom', fontsize=7, rotation=45)
    
    plt.tight_layout()
    plt.suptitle('Pixel coordinate differences analysis by point ID', fontsize=16, y=0.98)
    plt.show()

def plot_detailed_pixel_analysis(merged_df):
    """Detailed pixel difference analysis plot"""
    merged_df = merged_df.sort_values('ID')
    
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 12))
    
    ids = merged_df['ID'].astype(int)
    x_pos = range(len(ids))
    
    # Top left: X-axis difference scatter plot
    scatter1 = ax1.scatter(x_pos, merged_df['pixel_diff_X'], c=merged_df['pixel_diff_X'], 
                          cmap='Blues', s=60, alpha=0.7)
    ax1.set_xlabel('Point index')
    ax1.set_ylabel('X-axis pixel difference')
    ax1.set_title('X-axis pixel difference distribution')
    ax1.grid(True, alpha=0.3)
    plt.colorbar(scatter1, ax=ax1)
    
    # Top right: Y-axis difference scatter plot
    scatter2 = ax2.scatter(x_pos, merged_df['pixel_diff_Y'], c=merged_df['pixel_diff_Y'], 
                          cmap='Greens', s=60, alpha=0.7)
    ax2.set_xlabel('Point index')
    ax2.set_ylabel('Y-axis pixel difference')
    ax2.set_title('Y-axis pixel difference distribution')
    ax2.grid(True, alpha=0.3)
    plt.colorbar(scatter2, ax=ax2)
    
    # Bottom left: Difference heatmap
    im = ax3.imshow([[merged_df['pixel_diff_X'].values], [merged_df['pixel_diff_Y'].values]], 
                   cmap='hot', aspect='auto')
    ax3.set_xlabel('Point ID')
    ax3.set_ylabel('Axis')
    ax3.set_yticks([0, 1])
    ax3.set_yticklabels(['X-axis', 'Y-axis'])
    ax3.set_title('Pixel difference heatmap')
    plt.colorbar(im, ax=ax3)
    
    # Bottom right: Cumulative difference analysis
    sorted_diffs = np.sort(merged_df['pixel_diff_total'])
    y_vals = np.arange(1, len(sorted_diffs) + 1) / len(sorted_diffs)
    ax4.plot(sorted_diffs, y_vals, 'r-', linewidth=2)
    ax4.set_xlabel('Total pixel difference')
    ax4.set_ylabel('Cumulative probability')
    ax4.set_title('Pixel difference cumulative distribution')
    ax4.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.suptitle('Detailed pixel coordinate difference analysis', fontsize=16, y=0.98)
    plt.show()

def print_detailed_differences(merged_df):
    """Print detailed difference information for each point"""
    merged_df = merged_df.sort_values('ID')
    
    print("\n" + "="*90)
    print("Detailed pixel coordinate difference analysis for each point")
    print("="*90)
    print(f"{'ID':<5} {'File1(X,Y)':<15} {'File2(X,Y)':<15} {'ΔX':<8} {'ΔY':<8} {'Total_diff':<12}")
    print("-"*90)
    
    has_differences = False
    for _, row in merged_df.iterrows():
        id_val = int(row['ID'])
        x1, y1 = int(row['Pixel_X_file1']), int(row['Pixel_Y_file1'])
        x2, y2 = int(row['Pixel_X_file2']), int(row['Pixel_Y_file2'])
        dx, dy = row['pixel_diff_X'], row['pixel_diff_Y']
        total_diff = row['pixel_diff_total']
        
        if total_diff > 0:
            has_differences = True
            print(f"{id_val:<5} ({x1:>3},{y1:>3})      ({x2:>3},{y2:>3})      "
                  f"{dx:<8.1f} {dy:<8.1f} {total_diff:<12.2f}")
    
    if not has_differences:
        print("All points have identical pixel coordinates!")
    
    print("\n" + "="*90)
    print("Statistical summary:")
    print("="*90)
    print(f"Points compared: {len(merged_df)}")
    print(f"Points with differences: {len(merged_df[merged_df['pixel_diff_total'] > 0])}")
    print(f"Average X difference: {merged_df['pixel_diff_X'].mean():.3f} pixels")
    print(f"Average Y difference: {merged_df['pixel_diff_Y'].mean():.3f} pixels")
    print(f"Average total difference: {merged_df['pixel_diff_total'].mean():.3f} pixels")
    print(f"Maximum difference: {merged_df['pixel_diff_total'].max():.3f} pixels")
    print(f"Difference standard deviation: {merged_df['pixel_diff_total'].std():.3f} pixels")

def highlight_large_differences(merged_df, threshold=1.0):
    """Highlight points with large differences"""
    large_diffs = merged_df[merged_df['pixel_diff_total'] > threshold].sort_values('pixel_diff_total', ascending=False)
    
    if len(large_diffs) > 0:
        print(f"\n" + "="*70)
        print(f"Points with differences > {threshold} pixels (Total: {len(large_diffs)})")
        print("="*70)
        print(f"{'ID':<5} {'Total_diff':<12} {'X_diff':<10} {'Y_diff':<10}")
        print("-"*70)
        for _, row in large_diffs.iterrows():
            print(f"{int(row['ID']):<5} {row['pixel_diff_total']:<12.2f} "
                  f"{row['pixel_diff_X']:<10.1f} {row['pixel_diff_Y']:<10.1f}")

def main():
    # Read two data files
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
    
    # Calculate pixel coordinate differences
    print("Calculating pixel coordinate differences...")
    merged_df = calculate_pixel_differences(df1, df2)
    
    if len(merged_df) == 0:
        print("Warning: No matching IDs found!")
        return
    
    print(f"Found {len(merged_df)} matching points for comparison")
    
    # Print detailed differences
    print_detailed_differences(merged_df)
    
    # Highlight large differences
    highlight_large_differences(merged_df, threshold=0.5)
    
    # Generate plots
    print("Generating plots...")
    plot_pixel_differences_by_id(merged_df)
    plot_detailed_pixel_analysis(merged_df)
    
    print("Analysis completed!")

if __name__ == "__main__":
    main()