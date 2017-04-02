#include <iostream>

using namespace std;

int main()
{
    
    float pwm_duty_cycle = 0.2;
    
    float L1H = 1-pwm_duty_cycle;
    float L2H = 1;
    float L3H = 1;
    bool L1L = 0;
    bool L2L = 0;
    bool L3L = 1;
    
    cout << "L1H: " << L1H << endl
         << "L2H: " << L2H << endl
         << "L3H: " << L3H << endl
         << "L1L: " << L1L << endl
         << "L2L: " << L2L << endl
         << "L3L: " << L3L << endl<<endl;

    int8_t driveOut = 0x06;
    if (~driveOut & 0x02) L1H=1;
    if (~driveOut & 0x08) L2H=1;
    if (~driveOut & 0x20) L3H=1;

    L1L = L1L && (driveOut & 0x01);
    L2L = L2L && (driveOut & 0x04);
    L3L = L3L && (driveOut & 0x10);

    cout << "L1H: " << L1H << endl
         << "L2H: " << L2H << endl
         << "L3H: " << L3H << endl
         << "L1L: " << L1L << endl
         << "L2L: " << L2L << endl
         << "L3L: " << L3L << endl<<endl;

    
    L1L = driveOut & 0x01;
    L2L = driveOut & 0x04;
    L3L = driveOut & 0x10;

   if (driveOut & 0x02) L1H=(1-pwm_duty_cycle);
   if (driveOut & 0x08) L2H=(1-pwm_duty_cycle);
   if (driveOut & 0x20) L3H=(1-pwm_duty_cycle);
    
    cout << "L1H: " << L1H << endl
         << "L2H: " << L2H << endl
         << "L3H: " << L3H << endl
         << "L1L: " << L1L << endl
         << "L2L: " << L2L << endl
         << "L3L: " << L3L << endl;

    

}