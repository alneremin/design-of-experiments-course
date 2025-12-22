import csv

def save_to_csv(data, columns_names, output_path):
    with open(output_path, "a") as desc:
        writer = csv.DictWriter(desc, fieldnames=columns_names)
        writer.writerows(data)


def init_csv(columns_names, output_path):
    try:
        with open(output_path, "x") as desc:
            writer = csv.DictWriter(desc, fieldnames=columns_names)
            writer.writeheader()
    except FileExistsError:
        print(f"File '{output_path}' already exists.")