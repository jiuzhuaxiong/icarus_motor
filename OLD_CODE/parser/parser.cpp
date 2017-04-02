#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define ROTATION = 117;
#define FACTOR = 1000;

bool parseCmd(char* in, float& r, float& v);
bool isNaN(const float& val);
bool parseNote(char* in, int* note, int* duration, int& size);

int main(){
	char input[16];
	float r,v;
	int n[16];
	int d[16];
	int s = 0;

	while(true){
		printf("Input: \r\n");
		scanf("%s", input);
		printf("Inputted: %s\r\n", input);

		if(input[0] == 'q'){
			printf("Exiting\r\n");
			break;
		}

		if ( parseNote(input, n, d, s) ){
			for(int i=0; i<s; i++){
				printf("Note: %d, Duration: %d\r\n", n[i], d[i]);
			}
		}
		else if( !(parseCmd(input, r, v)) ){
			printf("Invalid input!\r\n");
		}
		else{
			if(isNaN(v)){
				printf("R: %f\r\n", r);
			}
			else if(isNaN(r)){
				printf("V: %f\r\n", v);
			}
			else if(isNaN(r) && isNaN(v)){
				printf("Invalid Command!\r\n");
			}
			else{
				printf("R: %f V: %f\r\n", r, v);
			}
		}
	}
	return 0;
}

bool parseCmd(char* in, float& r, float& v, bool& r_cmd, bool& v_cmd,){

	char buf_r[7] = {0};
	char buf_v[7] = {0};
	if((in[0] == 'R') && (strchr(in,'V') != NULL)){	// R and V command
		
		int pos;
		for(int i=0;i<=strlen(in);i++){
			if(in[i] == 'V'){
				pos = i;
			}
		}

		for(int i=1;i<pos;i++){
			buf_r[i-1] = in[i];
		}
		for(int i=pos+1;i<strlen(in);i++){
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
			v = 0 - v;
		}
		r_cmd = true;
		v_cmd = true;
		
		return true;
	}
	if(in[0] == 'V'){ // Only V command
		for(int i=1;i<=strlen(in);i++){
			buf_v[i-1] = in[i];
		}
		printf("buf_v: %s\r\n", buf_v);
		v = atof(buf_v);

		if (v>999.99 || v<-999.99){
			return false;
		}

		r_cmd = false;
		v_cmd = true;

		return true;
	}
	else if(in[0] == 'R'){ // Only R command
		for(int i=1;i<=strlen(in);i++){
			buf_r[i-1] = in[i];
		}
		printf("buf_r: %s\r\n", buf_r);
		r = atof(buf_r);

		if (r>999.99 || r<-999.99){
			return false;
		}

		r_cmd = true;
		v_cmd = false;

		return true;
	}
	return false;	// Used for invalid input commands, deal with at higher level
}

bool isNaN(const float& val){
	unsigned bval = *((unsigned*) &val);
	bool lo = (bval & 0x0FFF) != 0;
	bool hi = ((bval >> 24) & 0x7F) == 0x7F;
	if(lo && hi){return true;}
	return false;
}

bool parseNote(char* in, int* note, int* duration, int& size){
	int notes[] = {142, 127, 239, 213, 190, 179, 159}; // A B C D E F G
	int sharps[] = {134, 127, 225, 201, 179, 169, 150}; // A# B C# D# F F# G#
	int flats[] = {150, 134, 245, 225, 201, 190, 190}; //A^ B^ C^ D^ E^ E G^

	if(in[0] == 'T'){
		int i=1;
		int j=0;
		while(i<strlen(in)){
			if(in[i+1] == '#'){
				int idx = in[i] & 0x0F;
				note[j] = sharps[idx-1];
				duration[j] = int(in[i+2]-'0');
				i+=3;
				j+=1;
			}
			else if(in[i+1] == '^'){
				int idx = in[i] & 0x0F;
				note[j] = flats[idx-1];
				duration[j] = int(in[i+2]-'0');
				i+=3;
				j+=1;
			}
			else{
				int idx = in[i] & 0x0F;
				printf("Idx: %d\n", idx);
				note[j] = notes[idx-1];
				duration[j] = int(in[i+1]-'0');
				i+=2;
				j+=1;
			}
		}
		printf("Size: %d\n", j);
		size = j;
		return true;
	}
	return false;
}

