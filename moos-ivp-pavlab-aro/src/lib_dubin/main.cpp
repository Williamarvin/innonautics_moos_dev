#include "dubin.h"

#include <fstream>
#include <sstream>

using namespace std;

// int main(int ac, char* av[]) {

//     Point ps = Point(0, -20);
//     double hs = -M_PI/2;
//     // Point pg = Point(-5, 0);
//     // double hg = M_PI/2;
//     Point pg = Point(0, -80);
//     double hg = M_PI/2;
//     double r1 = 20;
//     double r2 = 20;
//     double r3 = 20;
//     //Need function for converting between compass and cartesian coordinates

//     DubinsPath dp = DubinsPath();
//     dp.findOptimalPath(ps, hs, pg, hg, r1, r2, r3);
//     dp.printDubinsPath();

//     string test = dp.findOptimalWaypoints(ps, hs, pg, hg, r1, r2, r3, 5);
//     cout << test << endl;

//     return 0;
// }

int main(int ac, char* av[]) {

    ifstream file("test.txt");
    string line;    
    double r1 = 1;
    double r2 = 1;
    double r3 = 1;
    //Need function for converting between compass and cartesian coordinates


    if(file.is_open()){
        while(getline(file, line)){
            stringstream ss(line);
            double xs, ys, hs, xg, yg, hg, length;
            string type;

            // Assuming each line is in the format: xs;ys;hs;xg;yg;hg;length;type
            char delimiter;  // To absorb the ';' characters

            ss >> xs >> delimiter >> ys >> delimiter >> hs >> delimiter >> xg >> delimiter >> yg >> delimiter >> hg >> delimiter >> length >> delimiter >> type;

            Point ps = Point(xs, ys);
            Point pg = Point(xg, yg);

            DubinsPath dp = DubinsPath();
            dp.findOptimalPath(ps, hs, pg, hg, r1, r2, r3);

            //Compare our calculated values stored in dp with length and type from file
            double epsilon = 0.0001;
            double diff = abs(length - dp.m_length);


            if ((diff > epsilon) || (type != dp.m_type)){
                cout << "Error: Length or type mismatch" << endl;
                cout << "\tLength of path: " << dp.m_length << " Type: " << dp.m_type << endl;
                cout << "\tLength of path: " << length << " Type: " << type << endl;
            }
        }

    } else {
        cout << "File not found" << endl;
    }

    return 0;
}