#include <stdio.h>

int main(){
  int i,j,k,l;
  int sum = 20;
  int count = 0, count2 = 0;
  for(i=0; i < 15; i++){
    for(j=0; j < 15; j++){
      //      if(i+j >= sum) break;
      for(k=0; k < 15; k++){
	//	if(i+j+k >= sum) break;
	for(l=0; l < 15; l++){
	  count2++;
	  if(i+j+k+l < sum){
	    printf("%d %d %d %d\n", i, j, k, l);
	    count++;
	  }
	  //	  else break;
	}
      }
    }
  }
  printf("count=%d\ncount2=%d\n", count, count2);
  return 1;
}
