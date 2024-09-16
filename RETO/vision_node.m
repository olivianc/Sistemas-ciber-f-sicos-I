setenv('ROS_DOMAIN_ID','0');
ros2('topic','list');
node =ros2node('/matlab_vision');
greenPub=ros2publisher(node,'/verde','std_msgs/Float64MultiArray');
BluePub=ros2publisher(node,'/azul','std_msgs/Float64MultiArray');
msgGr=ros2message(greenPub);
msgblu=ros2message(BluePub);
publishFrequency=50;
pause(1/publishFrequency);

% Crear el objeto webcam
cam = webcam(2); % Ajusta el índice si es necesario

% Obtener la lista de resoluciones disponibles
availableResolutions = cam.AvailableResolutions;

% Mostrar todas las resoluciones disponibles
disp('Resoluciones disponibles:');
disp(availableResolutions);5

% Seleccionar la resolución deseada si está disponible
desiredResolution = '1280x720';

% Inicializar arreglos para guardar posiciones y orientaciones
greenData = []; % Cada fila contendrá [X, Y, Z (Orientación)]
blueData = [];  % Cada fila contendrá [X, Y, Z (Orientación)]

figure; % Crear una nueva figura para mostrar el resultado en tiempo real

while true
    try
        % Capturar una imagen de la cámara
        img = snapshot(cam);

        % Convertir la imagen a espacio de color HSV
        hsvImg = rgb2hsv(img);

        % Definir el rango de color verde en el espacio HSV
        hueMinGreen = 0.2;  % Mínimo valor de tono para el verde
        hueMaxGreen = 0.5;  % Máximo valor de tono para el verde
        saturationMin = 0.3; % Saturación mínima
        saturationMax = 1.0; % Saturación máxima
        valueMin = 0.2;      % Brillo mínimo
        valueMax = 1.0;      % Brillo máximo

        % Crear una máscara binaria para el color verde
        greenMask = (hsvImg(:,:,1) > hueMinGreen) & (hsvImg(:,:,1) < hueMaxGreen) & ...
                    (hsvImg(:,:,2) > saturationMin) & (hsvImg(:,:,2) <= saturationMax) & ...
                    (hsvImg(:,:,3) >= valueMin) & (hsvImg(:,:,3) <= valueMax);

        % Eliminar ruidos pequeños en la máscara
        greenMask = bwareaopen(greenMask, 50);

        % Definir el rango de color azul en el espacio HSV
        hueMinBlue = 0.5;  % Mínimo valor de tono para el azul
        hueMaxBlue = 0.7;  % Máximo valor de tono para el azul

        % Crear una máscara binaria para el color azul
        blueMask = (hsvImg(:,:,1) > hueMinBlue) & (hsvImg(:,:,1) < hueMaxBlue) & ...
                   (hsvImg(:,:,2) > saturationMin) & (hsvImg(:,:,2) <= saturationMax) & ...
                   (hsvImg(:,:,3) >= valueMin) & (hsvImg(:,:,3) <= valueMax);

        % Eliminar ruidos pequeños en la máscara
        blueMask = bwareaopen(blueMask, 50);

        % Obtener propiedades de los objetos detectados en la máscara verde
        statsGreen = regionprops(greenMask, 'Centroid', 'Area', 'Orientation');
        % Obtener propiedades de los objetos detectados en la máscara azul
        statsBlue = regionprops(blueMask, 'Centroid', 'Area', 'Orientation');

        % Inicializar variables para la posición y orientación del objeto verde
        posGreen = [];
        orientationGreen = 0;
        

        % Inicializar variables para la posición y orientación del objeto azul
        posBlue = [];
        orientationBlue = 0;

        % Encontrar el objeto verde más grande
        if ~isempty(statsGreen)
            [~, idxGreen] = max([statsGreen.Area]);
            posGreen = statsGreen(idxGreen).Centroid;
            orientationGreen = statsGreen(idxGreen).Orientation; % Orientación del objeto verde más grande
            
            % Guardar los valores en el arreglo greenData

      
            msgGr.data=[posGreen(1);posGreen(2);orientationGreen];
            send(greenPub,msgGr);
            waitfor(publishFrequency);
           
        end

        % Encontrar el objeto azul más grande
        if ~isempty(statsBlue)
            [~, idxBlue] = max([statsBlue.Area]);
            posBlue = statsBlue(idxBlue).Centroid;
            orientationBlue = statsBlue(idxBlue).Orientation; % Orientación del objeto azul más grande
            
            % Guardar los valores en el arreglo blueData


            msgblu.data=[posBlue(1);posBlue(2);orientationBlue];
            send(BluePub,msgblu);
            waitfor(publishFrequency);
        
        end

        % Mostrar resultados
        imshow(img);
        hold on;
        if ~isempty(posGreen)
            % Mostrar la posición y orientación del objeto verde detectado
            fprintf('Posición del objeto verde: X = %.2f, Y = %.2f, Z (Orientación) = %.2f grados\n', posGreen(1), posGreen(2), orientationGreen);

            % Mostrar la imagen con la posición y orientación marcadas para el objeto verde
            plot(posGreen(1), posGreen(2), 'g*', 'MarkerSize', 15); % Marca verde

            % Dibujar una línea que indique la orientación del objeto verde
            angleGreen = deg2rad(orientationGreen); % Convertir a radianes
            lineLengthGreen = 50; % Longitud de la línea para la orientación
            xEndGreen = posGreen(1) + lineLengthGreen * cos(angleGreen);
            yEndGreen = posGreen(2) - lineLengthGreen * sin(angleGreen); % Coordenada y negativa por la orientación de la imagen

            % Dibujar la línea que representa la orientación
            line([posGreen(1), xEndGreen], [posGreen(2), yEndGreen], 'Color', 'g', 'LineWidth', 2);
        end

        if ~isempty(posBlue)
            % Mostrar la posición y orientación del objeto azul detectado
            fprintf('Posición del objeto azul: X = %.2f, Y = %.2f, Z (Orientación) = %.2f grados\n', posBlue(1), posBlue(2), orientationBlue);

            % Mostrar la imagen con la posición y orientación marcadas para el objeto azul
            plot(posBlue(1), posBlue(2), 'b*', 'MarkerSize', 15); % Marca azul

            % Dibujar una línea que indique la orientación del objeto azul
            angleBlue = deg2rad(orientationBlue); % Convertir a radianes
            lineLengthBlue = 50; % Longitud de la línea para la orientación
            xEndBlue = posBlue(1) + lineLengthBlue * cos(angleBlue);
            yEndBlue = posBlue(2) - lineLengthBlue * sin(angleBlue); % Coordenada y negativa por la orientación de la imagen

            % Dibujar la línea que representa la orientación
            line([posBlue(1), xEndBlue], [posBlue(2), yEndBlue], 'Color', 'b', 'LineWidth', 2);
        end

        hold off;

    catch ME
        % Si ocurre un timeout, mostrar el error y continuar
        disp('Error al capturar la imagen:');
        disp(ME.message);
        pause(0.5); % Pausar antes de reintentar
    end

    % Actualizar la ventana de forma continua
    drawnow;
end

% Limpiar recursos al terminar
clear cam;