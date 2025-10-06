#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

using namespace std;

int main() {
    string filename = "TSPA.csv"; // your CSV file
    char delimiter = ';';         // assuming semicolon-separated CSV

    // Vectors to store each column
    vector<int> x;
    vector<int> y;
    vector<int> costs;

    ifstream file(filename);
    if (!file.is_open()) {
        cerr << "Error: Could not open file " << filename << endl;
        return 1;
    }

    string line;
    int a, b, c;

    while (getline(file, line)) {
        stringstream ss(line);
        if (ss >> a >> delimiter >> b >> delimiter >> c) {
            x.push_back(a);
            y.push_back(b);
            costs.push_back(c);
        } else {
            cerr << "Warning: Skipping malformed line: " << line << endl;
        }
    }

    file.close();

    // Print the arrays to check
    cout << "x: ";
    for (int val : x) cout << val << " ";
    cout << endl;

    cout << "y: ";
    for (int val : y) cout << val << " ";
    cout << endl;

    cout << "costs: ";
    for (int val : costs) cout << val << " ";
    cout << endl;

    return 0;
}
