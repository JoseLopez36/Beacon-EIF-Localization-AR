# Este es un tipo de mensaje para las estadisticas para el proceso de localización mediante balizas y el filtro EIF
std_msgs/Header header

# Tiempos de ejecución - ns
float32 predict_time
float32 update_time
float32 filter_time

# Numero de medidas/calculos recibidos
int32 measurements_received
