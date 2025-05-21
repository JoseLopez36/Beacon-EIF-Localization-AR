# Este es un tipo de mensaje para las estadisticas para el proceso de localización mediante balizas y el filtro EIF
std_msgs/Header header

# Tiempos de ejecución - ns
int64 predict_time
int64 update_time
int64 filter_time

# Numero de medidas/calculos recibidos
int32 measurements_received
