#ifndef PARSE_H
#define PARSE_H

#include "globals.h"


bool parseCmd(char* in, float& r, float& v, bool& r_cmd, bool& v_cmd){
    char buf_r[7] = {0};
    char buf_v[7] = {0};
    char* v_pos = strchr(in,'V');
    
    char *r_val_start, *v_val_start;
    int r_val_len, v_val_len;
    
    if((in[0] == 'R') && (v_pos != NULL)){ // R and V command
        r_val_start = in + 1;
        r_val_len = v_pos - r_val_start;
        v_val_len = (in + strlen(in)) - (v_pos+1);

        r_cmd = true;
        v_cmd = true;
    }
    // Only R command
    else if(in[0] == 'R'){ 
        r_val_start = in + 1;
        r_val_len = (in + strlen(in)) - r_val_start;
 
        r_cmd = true;
        v_cmd = false;
    }
    // Only V command
    else if(in[0] == 'V'){ 
        v_val_start = in + 1;
        v_val_len = (in + strlen(in)) - v_val_start;
        
        r_cmd = false;
        v_cmd = true;
    }

    // Used for invalid input commands, deal with at higher level
    if(!r_cmd && !v_cmd) return false;

    if(r_cmd){
        // Copy the R value in buf_r
        memcpy( buf_r, r_val_start, r_val_len);
        r = atof(buf_r);
    }

    if(v_cmd){
        // Copy the V value in buf_v
        memcpy( buf_v, v_val_start, v_val_len);
        v = atof(buf_v);
        // Make the v positive if r was also specified
        if(v < 0.0 && r_cmd)     v = 0.0 - v;
    }

    return true;   
}


bool parseNote(char* in, uint8_t* note, uint8_t* duration, int8_t& size){

    if(in[0] == 'T'){
        int8_t i=1;
        int8_t j=0;
        while(i<strlen(in)){
            if(in[i+1] == '#'){
                uint8_t idx = in[i] & 0x0F;
                note[j] = sharps[idx-1];
                duration[j] = uint8_t(in[i+2]-'0');
                i+=3;
                j+=1;
            }
            else if(in[i+1] == '^'){
                uint8_t idx = in[i] & 0x0F;
                note[j] = flats[idx-1];
                duration[j] = uint8_t(in[i+2]-'0');
                i+=3;
                j+=1;
            }
            else{
                uint8_t idx = in[i] & 0x0F;
                note[j] = notes[idx-1];
                duration[j] = uint8_t(in[i+1]-'0');
                i+=2;
                j+=1;
            }
        }
        //pc.printf("Size: %d\n", j);
        size = j;
        return true;
    }
    return false;
}



#endif

