/************************************************************/
/*    NAME: Blake Cole                                      */
/*    ORGN: MIT                                             */
/*    FILE: ais_parse_playground.cpp                        */
/*    DATE: 30 MARCH 2021                                   */
/************************************************************/

#include <cmath>       /* pow */
#include <list>
#include <bitset>      /* bin2dec conversion */
#include <typeinfo>    /* query variable type */
#include <iostream>    /* Print to terminal      */

using namespace std;

string dec2bin(unsigned int n, unsigned int bits)
{
  /*
  string out = "";
  for(int i = bits-1; i >= 0; i--){
    int k = n >> i;
    if (k & 1)
      out.append("1");
    else
      out.append("0");
  }
  return(out);
  */
  string binary = bitset<6>(n).to_string();
  return(binary);
}


unsigned long bin2dec(string bin_str)
{
  /*
  cout << bin_str.c_str() << endl;
  uint64_t dbin = strtoull(bin_str.c_str(), NULL, 0);
  cout << "convert input string to integer: " << dbin << endl;
  uint64_t d = 0, i = 0, rem;
  while (dbin!=0){
    rem = dbin%10;
    dbin /= 10;
    d += rem*pow(2,i);
    cout << "d = " << d << endl;
    ++i;
  }
  return (d);
  */
  unsigned long i = -1;
  cout << "longest unsigned long: " << i << endl;
  unsigned long n = std::bitset<30>(bin_str).to_ulong();
  return(n);
}


void printList(list<int> mylist)
{
  cout << "mylist contains:";
  list<int>::iterator it;
  for (it=mylist.begin(); it!=mylist.end(); ++it)
    cout << ' ' << *it;
  cout << '\n' << endl;
}


int sgn(long int val) {
  return (0<val) - (val<0);
}

//--------------------------------------------------------------------------//
//--------------------------------------------------------------------------//
int main()
{
    //********************************************************************
  //--------------------------------------------------------------------
  cout << "----------------------------------" << endl;
  cout << "ASCII To Binary (6-bit) Conversion" << endl;
  string ascii = "15NO=dg000JrjadH?wsn=2dp00Sj"; //type1
  //string ascii = "B52Psw@00>fd@@63lhGQ3wi5oP06"; //type18
  //string ascii = "55NdvIT0Bk4iL@WSC;9A8t<60tpF00000000000U2h;26t0Ht4kQEl3l";
  //string ascii = "55NdvIT0Bk4iL@WSC;9A8t<60tpF00000000000U2h;26t0Ht4kQEl3lU84R@0000000000";
  //string ascii = "H5O51CP61@Dp000000000000000";
  cout << "ascii   = " << ascii << endl;
  string bin = "";
  cout << "decimal = ";
  for (int i=0; i<ascii.length(); i++){
    int d = int(ascii[i])-48;
    if (d > 40)
      d -= 8;
    bin.append(dec2bin(d,6));
    cout << d << " ";
  }
  cout << endl;
  cout << "binary  = " << bin << "\n" << endl;

  cout << "------------------------------------" << endl;
  cout << "Binary (6-bit) to Decimal Conversion" << endl;

  cout << "input = " << bin.substr(0,6) << endl;
  int type = bin2dec(bin.substr(0,6));
  cout << "message type = " << type << "\n" << endl;
  
  cout << "input = " << bin.substr(8,30) << endl;
  int mmsi = bin2dec(bin.substr(8,30));
  cout << "vessel mmsi  = " << mmsi << "\n" << endl;

  //cout << "----------------------------" << endl;
  //cout << "Decimal To Binary Conversion" << endl;
  //int n = 30;
  //int bits = 6;
  //string out = dec2bin(n, bits);
  //cout << "dec2bin(" << n << "," << bits << ") = " << out << endl;

  cout << "------------------------------------" << endl;
  cout << "Assigning 'unsigned long' to 'double'" << endl;
  unsigned long bitset_output = 3621;
  double m_var;
  m_var = bitset_output;
  cout << "unsigned long int: " << bitset_output << endl;
  cout << "double: " << m_var << " [" <<  typeid(m_var).name() << "]\n" << endl;

  cout << "------------------------------------" << endl;
  cout << "Decoding Vessel Name                " << endl;
  string payload = "000101000101011110101100111110011001100100000000010010110011000100110001011100010000100111100011010011001011001001010001001000111100001100000110000000111100111000010110000000000000000000000000000000000000000000000000000000000000000000100101000010110000001011000010000110111100000000011000111100000100110011100001010101110100000011110100100101001000000100100010010000000000000000000000000000000000000000000000000000000000000000";
  string m_vname = "";
  for (int i=0; i<20; ++i) {
    int index = 112 + 6*i;
    int decimal = bitset<6>(payload.substr(index,6)).to_ulong();
    cout << i << " : " << index << " : " << decimal << " : ";
    if (decimal > 0) {
      if (decimal <= 31)
        decimal += 64;
      char character = char(decimal);
      m_vname.push_back(character);
      cout << character << endl;
    }
    else {
      cout << endl;
    }
  }

  // remove whitespace from beginning and end
  string trim = " \t\n\r\f\v";
  m_vname.erase(0, m_vname.find_first_not_of(trim));
  m_vname.erase(m_vname.find_last_not_of(trim) + 1);
  cout << "VESSEL NAME: " << m_vname << "\n" << endl;


  cout << "------------------------------------" << endl;
  cout << "Two's Complement -> Signed Integer  " << endl;
  string bin_str = bin.substr(61,28);
  int N = bin_str.length();
  cout << bin_str << " [" << N << "]" << endl;
  unsigned long val = std::bitset<32>(bin_str).to_ulong();
  cout << "val = " << val << endl;
  cout << "2^(N-1) = " << pow(2,N-1) << endl;
  long int yy = sgn(pow(2,N-1)-val)*(pow(2,N-1)-abs(pow(2,N-1)-val));
  if ((yy == 0) && (val != 0))
    cout << -val << endl;
  else
    cout << yy << endl;
  cout << endl;

  cout << "------------------------------------" << endl;
  cout << "Size of Data Types                  " << endl;
  cout << "bool:              " << sizeof(bool) << " bytes" << endl;
  cout << "char:              " << sizeof(char) << " bytes" << endl;
  cout << "string:            " << sizeof(string) << " bytes" << endl;
  cout << "int:               " << sizeof(int)  << " bytes" << endl;
  cout << "short int:         " << sizeof(short int) << " bytes" << endl;
  cout << "long int:          " << sizeof(long int)  << " bytes" << endl;
  cout << "signed long int:   " << sizeof(signed long int) << " bytes" << endl;
  cout << "unsigned long int: " << sizeof(unsigned long int)<< " bytes" << endl;
  cout << "float:             " << sizeof(float) << " bytes" <<endl;
  cout << "double:            " << sizeof(double) << " bytes" << "\n" << endl;

  
  //********************************************************************
  //--------------------------------------------------------------------
  cout << "------------------------------------" << endl;
  cout << "List Iterator Playground            " << endl;
  list<int> mylist;
  list<int>::iterator it;
  for (int i=1; i<10; ++i) mylist.push_back(i*10);
  printList(mylist);

  cout << "step 1: shift iterator to desired position" << endl;
  int newint = 100;
  int nsteps = 6;
  it = mylist.begin();
  advance(it, nsteps);
  cout << "iterator position: " << nsteps << "\n" << endl;

  cout << "step 2: erase element using iterator" << endl;
  it = mylist.erase(it);
  printList(mylist);

  cout << "step 3: erase front item" << endl;
  mylist.pop_front();
  printList(mylist);

  cout << "step 4: insert new integer (" << newint << ") at iterator" << endl;
  mylist.insert(it, newint);
  printList(mylist);

  cout << "step 5: where is the iterator?" << endl;
  string at_end = "false";
  if (it==mylist.end())
    at_end = "true";
  cout << "iterator points to: " << *it << endl;
  cout << "it == mylist.end()? " << at_end << "\n" << endl;

  cout << "step 6: seek and destroy!" << endl;
  int kill = 100;
  cout << "integer to kill: " << kill << endl;
  list<int>::iterator itk = mylist.begin();
  while (itk != mylist.end()) {
    if (*itk == kill) {
      itk = mylist.erase(itk);
      continue;
    }
    ++itk;
  }
  printList(mylist);
  
}
