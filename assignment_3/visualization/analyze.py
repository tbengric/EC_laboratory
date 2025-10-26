import pandas as pd
import matplotlib.pyplot as plt
import io
import os

path_files = ["TSPA_paths.csv", "TSPB_paths.csv"]
full_files = ["../../data/TSPA.csv", "../../data/TSPB.csv"]
output_dir = "plots"

# Get the global MIN and MAX costs
all_nodes = {}
for file_path in full_files:
    df_full = pd.read_csv(file_path, sep=";", header=None, names=["x","y","cost","id"])
    base_name = os.path.splitext(os.path.basename(file_path))[0]
    all_nodes[base_name] = df_full

# Compute global min and max cost for all nodes
all_costs = pd.concat([df["cost"] for df in all_nodes.values()])
global_min_cost = all_costs.min()
global_max_cost = all_costs.max()


# Node size range
min_size, max_size = 200, 1300

# Precompute sizes for all nodes using global cost range
sizes_nodes_dict = {}
for name, df_nodes in all_nodes.items():
    sizes_nodes_dict[name] = min_size + (df_nodes['cost'] - global_min_cost) / (global_max_cost - global_min_cost) * (max_size - min_size)

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
    df_nodes = all_nodes[base_name]
    sizes_nodes = sizes_nodes_dict[base_name]

    print(f"Read file: {file_path}")

    # Plot for each method
    for algo_name, df in blocks.items():
        if df.empty:
            print(f"Skipping empty block: {algo_name}")
            continue

        fig, ax = plt.subplots(figsize=(20, 12))  

        # Sizes for the algorithm path nodes
        sizes_path = min_size + (df['cost'] - global_min_cost) / (global_max_cost - global_min_cost) * (max_size - min_size)

        # Plot all nodes as background
        ax.scatter(df_nodes.x, df_nodes.y, s=sizes_nodes, c='lightgray', edgecolor='black', linewidths=1, zorder=1, label='All nodes')

        # Plot edges of the algorithm path
        for i in range(len(df) - 1):
            x0, y0 = df.x.iloc[i], df.y.iloc[i]
            x1, y1 = df.x.iloc[i+1], df.y.iloc[i+1]
            ax.plot([x0, x1], [y0, y1], color='blue', linewidth=2.5, alpha=0.9)

        # Plot algorithm path nodes
        ax.scatter(df.x, df.y, s=sizes_path, c='skyblue', edgecolor='black', linewidths=1.2, zorder=2, label='Algorithm path')

        # Highlight start node
        start_node = df.iloc[0]
        start_size = sizes_path.iloc[0] if hasattr(sizes_path, 'iloc') else sizes_path
        ax.scatter(start_node.x, start_node.y, s=start_size, c='red', edgecolor='black', linewidths=1.2, zorder=4, label='Start node')

        # Add node IDs
        for i, row in df.iterrows():
            ax.text(row.x, row.y, str(row['id']), fontsize=10, color='black', ha='center', va='center', zorder=5)

        # Margins
        x_margin = (df_nodes.x.max() - df_nodes.x.min()) * 0.1
        y_margin = (df_nodes.y.max() - df_nodes.y.min()) * 0.1
        ax.set_xlim(df_nodes.x.min() - x_margin, df_nodes.x.max() + x_margin)
        ax.set_ylim(df_nodes.y.min() - y_margin, df_nodes.y.max() + y_margin)

        ax.set_aspect('equal')
        ax.set_title(f"{base_name} - {algo_name}", fontsize=14, weight='bold')
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=12)

        # Add best path
        best_path_text = f"Best Path: {' - '.join(list(df['id'].astype(str)))} "
        plt.figtext(0.5, 0.1, best_path_text, wrap=True, horizontalalignment='center', fontsize=12)

        # Save the plot
        safe_algo_name = algo_name.replace(" ", "_")
        output_file = os.path.join(output_dir, f"{base_name}_{safe_algo_name}.png")
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        plt.close()
        print(f"Saved plot: {output_file}")
