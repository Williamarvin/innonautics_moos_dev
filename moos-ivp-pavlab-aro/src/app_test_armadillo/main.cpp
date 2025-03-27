#include <iostream>
#include <armadillo>
 

using namespace std;
using namespace arma; 

#include <vector> 
#include <armadillo> 
#include <cmath>


int main(int ac, char * av[]) {
    

    cout << "<Hello Armadillo!>\n";
    arma::mat A(4, 5, fill::randu);
    arma::mat B(4, 5, fill::randu);
    
    cout << A*B.t() << endl;
    
    return 0;
}