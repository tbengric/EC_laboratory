import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import io
import os
import textwrap

path_files = ["assignment_1/visualization/TSPA_paths.csv", "assignment_1/visualization/TSPB_paths.csv"]
full_files = ["data/TSPA.csv", "data/TSPB.csv"]
output_dir = "assignment_1/visualization/plots"

# Get the global MIN and MAX costs
cost_limits = {}
for file_path in full_files:
    df_full = pd.read_csv(file_path, sep=";", header=None, names=["x","y","cost"])
    min_cost = df_full["cost"].min()
    max_cost = df_full["cost"].max()
    base_name = os.path.splitext(os.path.basename(file_path))[0]
    cost_limits[base_name] = (min_cost, max_cost)

# Go through each TSP type
for file_path in path_files:
    with open(file_path, "r") as f:
        content = f.read()

    blocks = {}
    current_name = None
    current_lines = []

    # Split into algorithms
    for line in content.splitlines():
        if not line.strip():
            continue
        if not line[0].isdigit() and not line.startswith("id,"):
            if current_name and current_lines:
                blocks[current_name] = pd.read_csv(io.StringIO("\n".join(current_lines)))
            current_name = line.strip()
            current_lines = []
        else:
            current_lines.append(line)

    if current_name and current_lines:
        blocks[current_name] = pd.read_csv(io.StringIO("\n".join(current_lines)))

    base_name = os.path.splitext(os.path.basename(file_path))[0].split("_")[0]
    min_cost, max_cost = cost_limits[base_name]
    print(f"Read file: {file_path}")

    # Plot for each method
    for algo_name, df in blocks.items():
        fig, ax = plt.subplots(figsize=(12, 10))  

        # Colors
        norm = plt.Normalize(min_cost, max_cost)
        cmap = cm.viridis
        node_colors = cmap(norm(df['cost']))
        cbar = plt.colorbar(
            cm.ScalarMappable(norm=norm, cmap=cmap),
            ax=ax,
            fraction=0.02, 
            pad=0.02       
        )
        cbar.set_label("Node Cost")

        # Edges
        for i in range(len(df) - 1):
            x0, y0 = df.x[i], df.y[i]
            x1, y1 = df.x[i + 1], df.y[i + 1]
            ax.plot([x0, x1], [y0, y1], color='gray', linewidth=1.2, alpha=0.8)
            mid_x, mid_y = (x0 + x1) / 2, (y0 + y1) / 2
            ax.text(mid_x, mid_y, str(i), fontsize=7, color='black', ha='center', va='center')

        # Last Edge
        ax.plot([df.x.iloc[-1], df.x.iloc[0]], [df.y.iloc[-1], df.y.iloc[0]], color='gray', linewidth=1.2, alpha=0.8)
        mid_x = (df.x.iloc[-1] + df.x.iloc[0]) / 2
        mid_y = (df.y.iloc[-1] + df.y.iloc[0]) / 2
        ax.text(mid_x, mid_y, str(len(df) - 1), fontsize=7, color='black', ha='center', va='center')

        # Make scatter
        sc = ax.scatter(df.x, df.y, s=120, c=node_colors, edgecolor='black', linewidths=0.7, zorder=3)
        
        x_margin = (df.x.max() - df.x.min()) * 0.1
        y_margin = (df.y.max() - df.y.min()) * 0.1
        ax.set_xlim(df.x.min() - x_margin, df.x.max() + x_margin)
        ax.set_ylim(df.y.min() - y_margin, df.y.max() + y_margin)

        ax.set_aspect('equal')
        ax.set_title(f"{base_name} - {algo_name}", fontsize=14, weight='bold')
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.grid(True, alpha=0.3)

        # Add best path
        best_path_text = f"Best Path: {' - '.join(list(df['id'].astype(str)))} "
        plt.figtext(0.5, 0.1, best_path_text, wrap=True, horizontalalignment='center', fontsize=12)

        # Save the plot
        safe_algo_name = algo_name.replace(" ", "_")
        output_file = os.path.join(output_dir, f"{base_name}_{safe_algo_name}.png")
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        plt.close()
        print(f"Saved plot: {output_file}")