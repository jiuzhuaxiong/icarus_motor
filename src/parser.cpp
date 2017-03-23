#include "parse.h"

bool parseCmd(char* in, float& r, float& v){

    char buf_r[7] = {0};
    char buf_v[7] = {0};
    if((in[0] == 'R') && (std::strchr(in,'V') != NULL)){    // R and V command
        
        int pos;
        for(int i=0;i<=std::strlen(in);i++){
            if(in[i] == 'V'){
                pos = i;
            }
        }

        for(int i=1;i<pos;i++){
            buf_r[i-1] = in[i];
        }
        for(int i=pos+1;i<std::strlen(in);i++){
            buf_v[i-pos-1] = in[i];
        }
        printf("buf_r: %s, buf_v: %s", buf_r, buf_v);

        r = atof(buf_r);
        v = atof(buf_v);

        if (r>999.99 || r<-999.99){
            return false;
        }
        if (v>999.99 || v<-999.99){
            return false;
        }

        if(v < 0){
            v *= -1;
        }
        return true;
    }
    if(in[0] == 'V'){ // Only V command
        for(int i=1;i<=std::strlen(in);i++){
            buf_v[i-1] = in[i];
        }
        printf("buf_v: %s\r\n", buf_v);
        v = atof(buf_v);

        if (v>999.99 || v<-999.99){
            return false;
        }

        memset((char*) &r, -1, sizeof(float)); // Used for when R does not change
        return true;
    }
    else if(in[0] == 'R'){ // Only R command
        for(int i=1;i<=std::strlen(in);i++){
            buf_r[i-1] = in[i];
        }
        printf("buf_r: %s\r\n", buf_r);
        r = atof(buf_r);

        if (r>999.99 || r<-999.99){
            return false;
        }

        memset((char*) &v, -1, sizeof(float)); // Used for when V does not change
        return true;
    }
    return false;   // Used for invalid input commands, deal with at higher level
}


bool isNaN(const float& val){
    unsigned bval = *((unsigned*) &val);
    bool lo = (bval & 0x0FFF) != 0;
    bool hi = ((bval >> 24) & 0x7F) == 0x7F;
    if(lo && hi){return true;}
    return false;
}