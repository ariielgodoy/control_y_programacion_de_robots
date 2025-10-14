# Control y Programación de Robots

Repositorio de prácticas de la asignatura **Control y Programación de Robots**, realizadas durante el curso académico 2025.  
El objetivo de estas prácticas es comprender y aplicar los fundamentos de **ROS 2**, la **comunicación entre nodos**, y el **control y localización de robots** mediante programación en **C++**.

---

## Contenido

### Práctica 1 – Publicación y suscripción en ROS 2
**Objetivos:**
- Introducir el modelo de comunicación **publisher/subscriber** en ROS 2.  
- Utilizar los mensajes estándar de los paquetes:
  - [`sensor_msgs`](https://docs.ros.org/en/noetic/api/sensor_msgs/html/index-msg.html)
  - [`geometry_msgs`](https://docs.ros.org/en/noetic/api/geometry_msgs/html/index-msg.html)
- Implementar un nodo que publique información de sensores y otro que la reciba y procese.

**Conceptos clave:**  
`rclcpp`, `Publisher`, `Subscription`, `callback`, `Node`.

---

### Práctica 2 – Generación de partículas con ruido
**Objetivos:**
- Introducir el concepto de **representación probabilística del estado del robot** mediante partículas.
- Implementar un generador de partículas que simule la **incertidumbre del posicionamiento** basada en la odometría.
- Añadir **ruido gaussiano** a las estimaciones de posición y orientación.

**Conceptos clave:**  
`odom`, `std::normal_distribution`, modelos de movimiento, simulación de ruido.

---

## Tecnologías utilizadas

- **ROS 2 Humble**  
- **C++**  
- **rclcpp**  
- **CoppeliaSim**  


