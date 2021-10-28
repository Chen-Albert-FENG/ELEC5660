function Optimal_path = path_from_A_star(map)
    Optimal_path = [];
    size_map = size(map,1);

    MAX_X=10;
    MAX_Y=10;
    MAX_Z=10;
    
    %Define the 3D grid map array.
    %Obstacle=-1, Target = 0, Start=1
    MAP=2*(ones(MAX_X,MAX_Y,MAX_Z));
    
    %Initialize MAP with location of the target
    xval=floor(map(size_map, 1));
    yval=floor(map(size_map, 2));
    zval=floor(map(size_map, 3));
    
    xTarget=xval;
    yTarget=yval;
    zTarget=zval;
    MAP(xval,yval,zval)=0;
    
    %Initialize MAP with location of the obstacle
    for i = 2: size_map-1
        xval=floor(map(i, 1));
        yval=floor(map(i, 2));
        zval=floor(map(i, 3));
        MAP(xval,yval,zval)=-1;
    end 
    
    %Initialize MAP with location of the start point
    xval=floor(map(1, 1));
    yval=floor(map(1, 2));
    zval=floor(map(1, 3));
    xStart=xval;
    yStart=yval;
    zStart=zval;
    MAP(xval,yval,zval)=1;
    
    % Main structure in the A* search =====================================================

    % Container storing nodes to be expanded, along with the f score (f=g+h)
    % Each node's (x,y,z) coordinate and its f score is stored in a row
    % For example, queue = [x1, y1, z1, f1; x2, y2, z2, f2; ...; xn, yn, zn, fn]
    queue = [];  

    % Arrays for storing the g score of each node, g score of undiscovered nodes is inf
    g = inf(MAX_X,MAX_Y,MAX_Z);

    % Arrays recording whether a node is expanded (popped from the queue) or not
    % expanded: 1, not expanded: 0
    expanded = zeros(MAX_X,MAX_Y,MAX_Z);

    % Arrays recording the parent of each node
    parents = zeros(MAX_X,MAX_Y,MAX_Z, 3);
    start = [xStart,yStart,zStart];
    tar = [xTarget,yTarget,zTarget];
    
    
    %Start your code here ==================================================================
    % TODO
    % find neighbors
    function ns = get_n(p,map)% 6 dimensions
        ns = [p(1)+1 p(2) p(3) ; ... 
                     p(1)-1 p(2) p(3) ; ... 
                     p(1) p(2)+1 p(3) ; ... 
                     p(1) p(2)-1 p(3) ; ... 
                     p(1) p(2) p(3)+1 ; ... 
                     p(1) p(2) p(3)-1 ]; 
        remove_idx = [];
        for idx = 1:size(ns,1) 
            if(min(ns(idx,:))<1 || max(ns(idx,:))>10 || map(ns(idx,1),ns(idx,2),ns(idx,3)) == -1)
                remove_idx=[remove_idx;idx];
            end
        end
        remove_idx = sort(remove_idx, 'descend');
        ns(remove_idx,:)=[];
    end
    % h(n)
    function h_v = h(curr,target)%Manhattan distance
        D = 1.0;
        h_v = D*sum(abs(target-curr));
    end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                 
    queue = [xStart,yStart,zStart,0];
    while 1 
        if size(queue,1) == 0
            disp("False!");
            break
        end
        [min_index,~] = find(queue(:,4) == min(queue(:,4)));
        cur = queue(min_index(1),1:3);
        g_n = queue(min_index(1),4);
        queue(min_index(1),:) = [];            
        expanded(cur(1),cur(2),cur(3)) = 1;
        if cur == tar
            final= [cur(1)-0.5,cur(2)-0.5,cur(3)-0.5];
            Optimal_path = [final;Optimal_path];
            while 1
                if cur == start
                    break
                end
                x = cur(1); 
                y = cur(2); 
                z = cur(3);
                cur = [parents(x,y,z,1),parents(x,y,z,2),parents(x,y,z,3)];
                Optimal_path = [cur;Optimal_path];

            end
        end    
        neighbors = get_n(cur,MAP);
           for neigh = 1:size(neighbors,1)
               temp_x = neighbors(neigh,1);
               temp_y = neighbors(neigh,2);
               temp_z = neighbors(neigh,3);
               temp = [temp_x,temp_y,temp_z];
               if expanded(temp_x,temp_y,temp_z) == 0
                   
                   if g(temp_x,temp_y,temp_z) == inf
                       queue = [queue;temp_x,temp_y,temp_z,g_n+1+h(temp, tar)];
                   end
                   if g(temp_x,temp_y,temp_z) > g_n + 1
                       g(temp_x,temp_y,temp_z) = g_n + 1;
                       parents(temp_x,temp_y,temp_z,1) = cur(1);
                       parents(temp_x,temp_y,temp_z,2) = cur(2);
                       parents(temp_x,temp_y,temp_z,3) = cur(3);
                   end
               end
           end 
    end
    
end
