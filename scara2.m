% Matriz de excentricidades: Define os comprimentos ao longo dos eixos
matrizExcentricidades = [ 1 0 0;  % Primeiro elo
                          1 0 0;  % Segundo elo
                          0 0 1;  % Deslocamento em Z (sem comprimento fixo)
                          0 0 0, ]; % terceiro elo

% Ordem das transformações: Três rotações em Z e um deslocamento em Z
ordem = {'z', 'z', 'Dz', 'z'};

% Calcular as transformadas homogêneas para a nova configuração
matrizNaBaseInercial = calcularTransformadas(ordem, matrizExcentricidades);

% Calcular os pontos das juntas a partir das transformações
pontosDeJuntas = calcularCinematicaTotal(matrizNaBaseInercial);

% Exibir o ponto final (cinemática direta)
dispCinematicaDireta(pontosDeJuntas{4});

% Matrizes de inércia para as três rotações em Z
Izero1 = [1 1 1;
          1 1 1;
          1 1 1];
Izero2 = Izero1;
Izero3 = Izero2;
Izero4 = Izero3;

% Lista completa de inércias
Izeros = {Izero1, Izero2, Izero3, Izero4};

% Calcular a dinâmica total do robô
Dinamica = calcularDinamicaTotal(pontosDeJuntas, Izeros);
