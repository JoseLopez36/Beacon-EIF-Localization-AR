# Este es un tipo de mensaje para las estadisticas para el proceso de localización mediante balizas y el filtro EIF
std_msgs/Header header

# Tiempos de ejecución - ns
float64 predict_time
float64 update_time
float64 filter_time

# Numero de medidas/calculos recibidos
int32 measurements_received

# Matriz y vector de informacion
float64[9] omega
float64[3] xi