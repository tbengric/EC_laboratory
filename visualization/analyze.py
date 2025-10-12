import matplotlib.pyplot as plt
import csv

def read_paths_with_cost(filename):
    paths = {}
    current_label = None
    with open(filename, 'r') as f:
        reader = csv.reader(f)
        for row in reader:
            if not row:
                continue
            # detect label
            if len(row) == 1 and not row[0].replace('.', '', 1).isdigit() and row[0] != "x,y,cost":
                current_label = row[0]
                paths[current_label] = []
                continue
            # skip header
            if row[0] == "x" and row[1] == "y":
                continue
            # only unpack if we have 3 values
            if len(row) == 3:
                x, y, cost = map(float, row)
                paths[current_label].append((x, y, cost))
            else:
                print("Skipping malformed line:", row)
    return paths

def plot_and_save_individual(paths, tsp_type):
    for label, nodes in paths.items():
        if not nodes:
            continue
        xs = [n[0] for n in nodes]
        ys = [n[1] for n in nodes]
        costs = [n[2] for n in nodes]

        plt.figure(figsize=(8,6))
        # Scatter plot: bigger dots, color represents cost
        plt.scatter(xs, ys, c=costs, cmap='viridis', s=150, edgecolor='black')
        # Connect nodes with a black line
        plt.plot(xs + [xs[0]], ys + [ys[0]], '-', color='gray', linewidth=0.85)
        plt.xlabel("X coordinate")
        plt.ylabel("Y coordinate")
        plt.title(f"{tsp_type}: {label}")
        plt.colorbar(label="Node Cost")
        filename = f"_{label.replace(' ', '_')}.png"
        plt.savefig("visualization/plots/"+ tsp_type+filename)
        plt.close()
        print(f"Saved plot: {tsp_type+filename}")

if __name__ == "__main__":
    tsp_type = "TSPB"
    paths = read_paths_with_cost("visualization/" + tsp_type+"_paths.csv")
    plot_and_save_individual(paths, tsp_type)
