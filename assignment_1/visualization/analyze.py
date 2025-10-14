import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import numpy as np
import io
import os

# === Lista plików do przetworzenia ===
files = ["assignment_1/visualization/TSPA_paths.csv", "assignment_1/visualization/TSPB_paths.csv"]
output_dir = "assignment_1/visualization/plots"
os.makedirs(output_dir, exist_ok=True)

for file_path in files:
    with open(file_path, "r") as f:
        content = f.read()

    # === Split file by algorithm names ===
    blocks = {}
    current_name = None
    current_lines = []

    for line in content.splitlines():
        if not line.strip():
            continue
        if not line[0].isdigit() and not line.startswith("x,"):
            if current_name and current_lines:
                blocks[current_name] = pd.read_csv(io.StringIO("\n".join(current_lines)))
            current_name = line.strip()
            current_lines = []
        else:
            current_lines.append(line)

    if current_name and current_lines:
        blocks[current_name] = pd.read_csv(io.StringIO("\n".join(current_lines)))

    print(f"Loaded algorithms from {file_path}: {list(blocks.keys())}")

    # === Plot each algorithm separately and save ===
    for name, df in blocks.items():
        fig, ax = plt.subplots(figsize=(8, 6))
        
        # colormap based on node cost
        norm = plt.Normalize(df['cost'].min(), df['cost'].max())
        cmap = cm.viridis
        node_colors = cmap(norm(df['cost']))
        
        # draw edges with numbers
        for i in range(len(df) - 1):
            x0, y0 = df.x[i], df.y[i]
            x1, y1 = df.x[i+1], df.y[i+1]
            ax.plot([x0, x1], [y0, y1], color='gray', linewidth=1.5)
            
            # show edge number
            mid_x, mid_y = (x0 + x1)/2, (y0 + y1)/2
            ax.text(mid_x, mid_y, str(i), fontsize=7, color='black', ha='center', va='center')
        
        # close the cycle
        ax.plot([df.x.iloc[-1], df.x.iloc[0]], [df.y.iloc[-1], df.y.iloc[0]], color='gray', linewidth=1.5)
        mid_x = (df.x.iloc[-1] + df.x.iloc[0])/2
        mid_y = (df.y.iloc[-1] + df.y.iloc[0])/2
        ax.text(mid_x, mid_y, str(len(df)-1), fontsize=7, color='black', ha='center', va='center')

        # plot nodes colored by cost
        sc = ax.scatter(df.x, df.y, color=node_colors, s=80, edgecolor='black', zorder=3)
        
        # annotate nodes with IDs
        for i, node_id in enumerate(df['id']):
            ax.text(df.x[i], df.y[i]-10, str(node_id), fontsize=8, ha='center', va='top', zorder=4)
        
        # axis and grid
        ax.set_title(name)
        ax.axis('equal')
        ax.set_xlim(0, df.x.max())
        ax.set_ylim(0, df.y.max())
        ax.grid(True)
        
        # add colorbar for cost
        cbar = plt.colorbar(cm.ScalarMappable(norm=norm, cmap=cmap), ax=ax)
        cbar.set_label('Node Cost')
        
        # add extra info below the plot
        total_cost = df['cost'].sum()
        num_nodes = len(df)
        fig.text(
            0.5, 0.01,
            f"Total nodes: {num_nodes} | Sum of costs: {total_cost}",
            ha='center', fontsize=10
        )
        
        # save plot
        filename = f"{output_dir}/{name.replace(' ', '_')}.png"
        plt.savefig(filename, dpi=300, bbox_inches='tight')
        plt.close()
        print(f"Saved plot: {filename}")

