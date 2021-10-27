function s_des = trajectory_generator(t, path, h)

persistent P
persistent t_p
persistent des_p

if nargin > 1 % pre-process can be done here (given waypoints)
    total_time = 25.0;
    interval = path(2:end,:) - path(1:end-1,:);
    norm = vecnorm(interval')';
    cum_norm = cumsum(norm);
    t_p = total_time * cum_norm/cum_norm(end);
    t_p = [0;t_p];
    
    N = 8;%minimum snap
    K = length(path) - 1;% number of segments
    Q = zeros(N*K, N*K);
    P = zeros(N*K, 3);
    all_c = 6*K; % the number of all constraints
    sel_c = 3*K+3; % the number of selected constraints
    f_num = K+5; % fixed d
    M = zeros(all_c, N*K); % Mapping matrix 
    C = zeros(all_c, sel_c); % Selection matrix
    d_x = zeros(f_num, 1);
    d_y = zeros(f_num, 1);
    d_z = zeros(f_num, 1);
    
    for q = 1:K
        for r = 5:N
            for c = 5:N
                Q((q-1)*N+r,(q-1)*N+c) = factorial(r-1)*factorial(c-1)*(t_p(q+1)^(r+c-9) - t_p(q)^(r+c-9))/(factorial(r-5)*factorial(c-5)*(r+c-9));
            end
        end
    end
    
    for k = 1:K% M matrix
        M(2*(k-1)+1,N*(k-1)+1:N*k) = [1,t_p(k),t_p(k)^2,t_p(k)^3,t_p(k)^4,t_p(k)^5,t_p(k)^6,t_p(k)^7];
        M(2*k,N*(k-1)+1:N*k) = [1,t_p(k+1),t_p(k+1)^2,t_p(k+1)^3,t_p(k+1)^4,t_p(k+1)^5,t_p(k+1)^6,t_p(k+1)^7];% position
        M(2*K+2*k-1,N*(k-1)+1:N*k) = [0,1,2*t_p(k),3*t_p(k)^2,4*t_p(k)^3,5*t_p(k)^4,6*t_p(k)^5,7*t_p(k)^6]; 
        M(2*K+2*k,N*(k-1)+1:N*k) = [0,1,2*t_p(k+1),3*t_p(k+1)^2,4*t_p(k+1)^3,5*t_p(k+1)^4,6*t_p(k+1)^5,7*t_p(k+1)^6];%velocity
        M(4*K+2*k-1,N*(k-1)+1:N*k) = [0,0,2,6*t_p(k),12*t_p(k)^2,20*t_p(k)^3,30*t_p(k)^4,42*t_p(k)^5]; 
        M(4*K+2*k,N*(k-1)+1:N*k) = [0,0,2,6*t_p(k+1),12*t_p(k+1)^2,20*t_p(k+1)^3,30*t_p(k+1)^4,42*t_p(k+1)^5];%acceleration
    end
    
    matching = zeros(2*K-2,K-1);
    for i = 1:K-1
        matching(2*i-1,i) = 1;
        matching(2*i,i) = 1;
    end
    
    C(1,1) = 1;
    C(2*K,K+1) = 1;
    C(2:2*K-1,2:K) = matching;%C matrix position
    
    C(2*K+1,K+2) = 1;
    C(4*K,K+3) = 1;
    C(2*K+2:4*K-1,K+6:2*K+4) = matching;%C matrix velocity
    
    C(4*K+1,K+4) = 1;
    C(6*K,K+5) = 1;
    C(4*K+2:6*K-1,2*K+5:3*K+3) = matching;%C matrix acceleration
    
    K_m = pinv(M)*C;%transform matrix KtQK
    R = K_m'*Q*K_m;
    
    d_x(1:K+1) = path(1:end,1);% x_position
    d_y(1:K+1) = path(1:end,2);% y_position
    d_z(1:K+1) = path(1:end,3);% z_position
    % d_x(K+2:end) = 0;% x_velocity and x_acceleration
    % d_y(K+2:end) = 0;% y_velocity and y_acceleration
    % d_z(K+2:end) = 0;% z_velocity and z_acceleration
    
    pp = R(f_num+1:end,f_num+1:end);
    fp = R(1:f_num,f_num+1:end);
    d_p_x = -inv(pp)*fp'*d_x;
    d_p_y = -inv(pp)*fp'*d_y;
    d_p_z = -inv(pp)*fp'*d_z;
    
    P(:,1) = K_m*[d_x;d_p_x];
    P(:,2) = K_m*[d_y;d_p_y];
    P(:,3) = K_m*[d_z;d_p_z];
else % output desired trajectory here (given time)
    s_des = zeros(11,1);
    if t > t_p(end)
        s_des(1:3) = des_p;
        return
    end
    
    for ti=1:size(t_p,1)
        if t>t_p(ti) && t<t_p(ti+1)
            time = t_p(ti);
            idx = ti;
        end
    end
    
    param = P((8*(idx-1)+1):(8*idx),:);
    s_des(1:3) = [1,t,t^2,t^3,t^4,t^5,t^6,t^7]*param;
    des_p = s_des(1:3);
    s_des(4:6) = [0,1,2*t,3*t^2,4*t^3,5*t^4,6*t^5,7*t^6]*param;
    s_des(7:9) = [0,0,2,6*t,12*t^2,20*t^3,30*t^4,42*t^5]*param;    
% s_des(1:3) desire position
% s_des(4:6) desire velocity
% s_des(7:9) desire acceleration
% s_des(10) desire yaw
% s_des(11) desire yaw rate

end

end


