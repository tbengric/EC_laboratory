import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path

# CSV files are in similarity_plots directory
csv_dir = Path("similarity_plots")

# Output plots go to plots directory
plots_dir = Path("plots")
plots_dir.mkdir(parents=True, exist_ok=True)

tsp_types = ["TSPA", "TSPB"]
similarity_types = [
    ("avg_similarity_edges", "Average Similarity", "Common Edges"),
    ("avg_similarity_nodes", "Average Similarity", "Common Nodes"),
    ("best_similarity_edges", "Similarity to Best Local Optimum", "Common Edges"),
    ("best_similarity_nodes", "Similarity to Best Local Optimum", "Common Nodes"),
    ("ils_similarity_edges", "Similarity to ILS Solution", "Common Edges"),
    ("ils_similarity_nodes", "Similarity to ILS Solution", "Common Nodes")
]

for tsp in tsp_types:
    for sim_file, title_prefix, ylabel in similarity_types:
        csv_file = csv_dir / f"{tsp}_{sim_file}.csv"
        
        if not csv_file.exists():
            print(f"File not found: {csv_file}")
            continue
        
        # Read data
        df = pd.read_csv(csv_file)
        
        # Create scatter plot
        plt.figure(figsize=(10, 6))
        plt.scatter(df['objective'], df['similarity'], alpha=0.5, s=20)
        
        # Add trend line
        #z = np.polyfit(df['objective'], df['similarity'], 1)
        #p = np.poly1d(z)
        #plt.plot(df['objective'], p(df['objective']), "r--", alpha=0.8, linewidth=2)
        
        # Calculate correlation
        corr = df['objective'].corr(df['similarity'])
        
        plt.xlabel('Objective Function Value', fontsize=12)
        plt.ylabel(ylabel, fontsize=12)
        plt.title(f'{tsp}: {title_prefix}\nCorrelation: {corr:.4f}', fontsize=14)
        plt.grid(True, alpha=0.3)
        
        # Save plot
        output_file = plots_dir / f"{tsp}_{sim_file}.png"
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        plt.close()
        
        print(f"Generated: {output_file}")

print("\nAll plots generated successfully!")
