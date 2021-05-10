double kp = -0.2, ki = 0, kd = 0;  //Constantes pour le controlleur PID
double previous_error;

double PID(double currentValue, double desiredValue) {
  double error = desiredValue - currentValue;;
  double derivative = error - previous_error;
  static double Integrated_error = 0;
  Integrated_error=Integrated_error*0.8+ error*0.2;
  return kp*error+ki*Integrated_error+kd*derivative;
  //    Proportionnelle   Intégrale      Dérivée
  //PID
}
