#include <stdio.h>
#include <unistd.h>

#define UNUSED __attribute__((unused))

int main(UNUSED int argc, UNUSED char const *argv[]){
	
	char str[] = "abcd";
	int len = 4;
	write(1, str, len);
	//printf("Hello World from User : %s\n", argv[1]);
	return 0;
}
