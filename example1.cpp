#include<iostream>
#include<vector>
#include<math.h>
// using std::cout;
using namespace std;


int main(){
    vector< vector<double> > a (25, vector<double>(25,0));
    for(int i = 0; i<a.size(); i++){
        for(int j = 0; j<a[0].size(); j++){
            a[i][j] = sqrt( pow(i-13,2) + pow(j-24,2));
            cout<<a[i][j]<<"\t";
        }
        cout<<endl;
    }

}