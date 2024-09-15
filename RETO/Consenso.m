%% Conexión con el robot

% Inicializa el entorno ROS 2 y el ID de dominio
setenv('ROS_DOMAIN_ID', '0');
% Lista los tópicos de ROS 2 para confirmar la conexión
ros2('topic', 'list');
% Crea un nodo de ROS 2
node = ros2node('/matlab_node');                                                                                                                                 
% Crea un publisher para el tópico /cmd_vel
cmdVelPub = ros2publisher(node, '/tb3_5/cmd_vel', 'geometry_msgs/Twist');
% Crea un subscriber para el tópico /verde
BSub = ros2subscriber(node, '/azul', 'std_msgs/Float64MultiArray');

% Crea un mensaje Twist
msg = ros2message(cmdVelPub);
% Configuración inicial
Tf = 50; % Tiempo final de simulación
t = 0;
dt = 0.01;

% Obtener datos del tópico verde
PoseB = receive(BSub);  % Tiempo de espera de 2 segundos
data = PoseB.data;
% Asegúrate de que los datos tienen al menos 3 elementos
if numel(data) >= 3
    x1 = data(1);
    y1 = data(2);
    theta1 = data(3);
else
    error('Los datos del tópico verde no tienen el formato esperado.');
end

disp(['Posición inicial Robot 2: [', num2str(x1), ', ', num2str(y1), '], Orientación: ', num2str(theta1)]);
xL = 1610;  
yL = 72;  
thetaL = pi;

x2 = 1668;  
y2 = 223;  
theta2 = 0;
vL=0.000005;

% Posición final deseada
x_deseado = 1613;
y_deseado = 74;

% Parámetros de control
kpd = 0.001; % Ganancia proporcional para la distancia
kpr = 0.4*0.02; % Ganancia proporcional para la rotación
vmax = 0.01; % Velocidad máxima lineal
wmax = 0.01; % Velocidad angular máxima
d_d = 0.5; % Distancia deseada de 3 unidades entre los robots y el líder
umbral_distancia = 15; % Distancia umbral para detener el movimiento (ajusta según sea necesario)

Thetae = [];
i = 1;

%% Inicialización para la gráfica en tiempo real
figure;
hold on;
grid on;
xlabel('Posición X');
ylabel('Posición Y');
title('Trayectoria del Robot 1 en Tiempo Real');
xlim([-2000, 2000]);  % Ajusta estos límites según sea necesario
ylim([-2000, 2000]);

% Inicializa las variables para almacenar las posiciones en X e Y
posX = [];
posY = [];

% Inicialización de la gráfica en tiempo real
robot1_trayectoria = plot(posX, posY, 'b-', 'LineWidth', 1.5);  % Trayectoria Robot 1

%% Loop principal
while t < Tf
    % Obtener datos del tópico verde
    PoseB = receive(BSub);  % Tiempo de espera de 2 segundos
    data = PoseB.data;
    % Asegúrate de que los datos tienen al menos 3 elementos
    if numel(data) >= 3
        x1 = data(1);
        y1 = data(2);
        theta1 = data(3);
    else
        error('Los datos del tópico verde no tienen el formato esperado.');
    end

    % Actualizar la posición del robot 1 en las listas de trayectoria
    posX = [posX, x1];
    posY = [posY, y1];

    % Actualizar la gráfica en tiempo real
    set(robot1_trayectoria, 'XData', posX, 'YData', posY);

    % Actualización de la posición del líder
    xL = xL + vL * cos(thetaL) * dt;
    yL = yL + vL * sin(thetaL) * dt;

    % Actualización de la orientación del líder hacia el punto final
    thetaL = atan2(y_deseado - yL, x_deseado - xL);  %% revisar signo 

    % Cálculo de la distancia euclidiana de los robots al líder
    Pe1 = [x1; y1] - [xL; yL]; % Vector de posición del robot 1 al líder
    d1 = norm(Pe1);            % Distancia euclidiana del robot 1 al líder
    de1 = d1 - d_d;             % Error de distancia del robot 1 al líder

    Pe2 = [x2; y2] - [xL; yL]; % Vector de posición del robot 2 al líder
    d2 = norm(Pe2);            % Distancia euclidiana del robot 2 al líder
    de2 = d2 - d_d;             % Error de distancia del robot 2 al líder

    % Cálculo de la distancia a la posición deseada
    distancia_robot1_a_deseado = norm([x1 - x_deseado, y1 - y_deseado]);
    distancia_robot2_a_deseado = norm([x2 - x_deseado, y2 - y_deseado]);

    % Verifica si ambos robots están cerca del punto deseado
    if distancia_robot1_a_deseado < umbral_distancia && distancia_robot2_a_deseado < umbral_distancia
        % Detiene ambos robots
        msg.linear.x= 0;
        msg.angular.y = 0;
        V2 = 0;
        w2 = 0;
    else
        % Velocidades deseadas hacia el líder basadas en la distancia
        V1d = -(Pe1 / norm(Pe1)) * kpd * de1;
        V2d = -(Pe2 / norm(Pe2)) * kpd * de2;

        % Cálculo de referencias de orientación
        theta1d = atan2(V1d(2), V1d(1));
        theta2d = atan2(V2d(2), V2d(1));

        % Cálculo de errores de orientación
        thetae1 = theta1 - theta1d;
        if thetae1 > pi
            thetae1 = thetae1 - 2*pi;
        elseif thetae1 < -pi
            thetae1 = thetae1 + 2*pi;
        end

        thetae2 = theta2 - theta2d;
        if thetae2 > pi
            thetae2 = thetae2 - 2*pi;
        elseif thetae2 < -pi
            thetae2 = thetae2 + 2*pi;
        end

        % Leyes de control
        V1 = norm(V1d);
        if V1 > vmax
            V1 = vmax;
        end
        w1 = -kpr * thetae1;

        V2 = norm(V2d);
        if V2 > vmax
            V2 = vmax;
        end
        w2 = -kpr * thetae2;
    end

    msg.linear.x = -V1;
    msg.angular.z = -w1;
    send(cmdVelPub, msg);

    % Simulación de la posición y orientación del robot 1
    x1p = V1 * cos(theta1);
    y1p = V1 * sin(theta1);
    theta1p = w1;

    % Actualización de las posiciones
    x1 = x1 + x1p * dt;
    y1 = y1 + y1p * dt;
    theta1 = theta1 + theta1p * dt;

    % Pausa para la animación
    pause(0.01);
end

% Establece las velocidades en cero para detener el robot
msg.linear.x = 0.0;
msg.angular.z = 0.0;
% Envía el mensaje de paro
send(cmdVelPub, msg);
disp('Comando de paro enviado');

% Pausa un momento para asegurar que el mensaje de paro sea procesado
pause(10);

% Limpieza: Limpia el nodo y el publisher
clear cmdVelPub node;

% Muestra un mensaje indicando que el script ha finalizado
disp('Nodo y publisher han sido limpiados. Saliendo.')