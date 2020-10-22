int mapInteger(int n, int inMin, int inMax, int outMin, int outMax) {
 return (n - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

double mapDouble(double n, double inMin, double inMax, double outMin, double outMax) {
 return (n - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

int constrain(int n, int min, int max) {
 if(n < min)
  n = min;
 else if(n > max)
  n = max;

 return n;
}

double constrain(double n, double min, double max) {
 if(n < min)
  n = min;
 else if(n > max)
  n = max;

 return n;
}
