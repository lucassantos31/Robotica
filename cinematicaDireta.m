function cinematica = cinematicaDireta(robo, elo)
    

    
    cinematica = struct();
    cinematica.n = robo.n;
    cinematica.Lcm = [sym(sprintf('Lxcm%d', elo), 'real'), sym(sprintf('Lycm%d', elo), 'real'), sym(sprintf('Lzcm%d', elo), 'real')];
    
    % Extração correta das variáveis
    cinematica.q   = robo.juntas(1,1:elo); % Posições articulares
    cinematica.dq  = robo.juntas(2,1:elo); % Velocidades articulares
    cinematica.d2q = robo.juntas(3,1:elo); % Acelerações articulares
    

    % Verifica se os tamanhos estão corretos
    if length(cinematica.q) ~= elo || length(cinematica.dq) ~= elo || length(cinematica.d2q) ~= elo
        error('Erro na extração das variáveis! Verifique os índices.');
    end

    % Armazena as juntas corretamente na estrutura
    cinematica.juntas = [cinematica.q; cinematica.dq; cinematica.d2q];

    % Outros cálculos de cinemática
    cinematica.L      = robo.L(1:elo,3);
    cinematica.T      = simplify(robo.T{elo});
    cinematica.nome   = ['P', num2str(elo)];
    cinematica.P      = simplify(robo.T{elo}(1:3, 4));
    cinematica.JL     = simplify(jacobian(transpose(cinematica.P), cinematica.q));
    cinematica.V      = simplify(cinematica.JL * transpose(cinematica.dq));
    cinematica.dJL    = DerivadaJacobiano(cinematica.JL, cinematica.q, cinematica.dq);
    cinematica.A      = simplify(cinematica.JL * transpose(cinematica.d2q) + cinematica.dJL * transpose(cinematica.dq));
    cinematica.JA     = calcularJacobianoAngular(robo, elo);
    cinematica.W      = cinematica.JA * transpose(cinematica.dq);
    cinematica.dJA    = DerivadaJacobiano(cinematica.JA, cinematica.q, cinematica.dq);
    cinematica.alfa   = cinematica.JA * transpose(cinematica.d2q) + cinematica.dJA * transpose(cinematica.dq);
    
    if elo == 1
        cinematica.Pcm    = [cinematica.T(1:4, 1:3),[0;0;0;1]] * Deslocamento(cinematica.Lcm);
    else
        cinematica.Pcm    = [cinematica.T(1:4,1:3),[robo.T{elo-1}(1:3, 4);1]] * Deslocamento(cinematica.Lcm);
    end
    cinematica.Pcm    = simplify(cinematica.Pcm(1:3,4));
    cinematica.JLcm   = simplify(jacobian(transpose(cinematica.Pcm), cinematica.q));
    cinematica.Vcm    = simplify(cinematica.JLcm * transpose(cinematica.dq));
    cinematica.dJLcm  = DerivadaJacobiano(cinematica.JLcm, cinematica.q, cinematica.dq);
    cinematica.Acm    = simplify(cinematica.JLcm * transpose(cinematica.d2q) + cinematica.dJLcm * transpose(cinematica.dq));
end