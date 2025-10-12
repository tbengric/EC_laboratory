import pandas as pd

filename = "solution_checker/Solution checker.xlsx"

for tsp_type in ["TSPA", "TSPB"]:
    df = pd.read_excel(filename, sheet_name=tsp_type)

    df_path = df[pd.notna(df["List of nodes"])]
    path_list = df_path["List of nodes"].astype(int).tolist()

    print("Path from solution checker:", path_list[:-1])

    with open(f"solution_checker/selected_nodes/{tsp_type}.txt", "w") as f:
        for node in path_list[:-1]:
            f.write(f"{node}\n")