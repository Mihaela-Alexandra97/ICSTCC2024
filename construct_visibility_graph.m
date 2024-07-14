function [Nodes,Adj] = construct_visibility_graph(Obstacles,Cities,init_fig_handle)
%Obstacles{i}.vertices is a matrix with two rows, vertices on columns - not yet closed
%  all obstacles should be convex (points are in order) and disjoint
%Cities is a matrix with two rows, containing points to be visited
%Nodes will be a matrix with two rows, where column i gives the coordinates of node i
%Adj is a symmetric adjacency matrix, with costs = Euclidean distance, and 0 where there is no arc
%  the main diagonal will be with 0
% init_fig_handle is handle or figure, to plot visibility line segments


Nodes = Cities; %initial and all points to be visited
polygs =[];   %will store all closed obstacles, for testing visibility (intersections)
node_to_obs = zeros(1,size(Nodes,2));   %node_to_obs(i) will give the obstacle number to which node i belongs, or zero if it's a city
for i=1:length(Obstacles)
    Nodes = [Nodes , Obstacles{i}.vertices];    %all vertices are points in graph
    node_to_obs = [node_to_obs , i*ones(1,size(Obstacles{i}.vertices,2))];
    polygs = [polygs, Obstacles{i}.vertices(:,[1:end 1]), [NaN;NaN] ];  %close each obstacle and separate with NaN
end
polygs(:,end) = []; %remove last column with NaN

n_nodes = size(Nodes,2);    %number of nodes
Adj=zeros(n_nodes); %initialize with unconnected

figure(init_fig_handle);
for i=1:n_nodes-1
    for j=i+1:n_nodes
        segm = Nodes(:,[i,j]); %coordinates of points defining the current segment to test
        [xi,yi] = polyxpoly(segm(1,:),segm(2,:),polygs(1,:),polygs(2,:));
        int_p = [xi , yi]'; %intersection points, on columns 
        % for overlapping edges, only extremities are given, so can appear edes (good) and diagonals in the same obstacle (bad)
        % the diagonals will be removed when testing supporting and separating arcs (diagonals are neither)
        int_p = setdiff(int_p',segm','rows')';   %remove extremities
        if isempty(int_p)   %nodes i and j are visible (or diagonals)
            %test if supporting or separating
            % for line given by points A and B and query point P,
            % "sign((xB - xA) * (yP - yA) - (yB - yA) * (xP - xA))" is 0 if colinear, -1 if on one side, +1 if on the other
            % it is basicaly the sign of the z-component of cross product between vectors AB and AP
            xA = Nodes(1,i); yA = Nodes(2,i); xB = Nodes(1,j); yB = Nodes(2,j);
            sup_sep = 1;    %assume good edge, modify below if necessary
            for k=[i,j]
                c_obs = node_to_obs(k); %obstacle to which current node (i, then j) belongs (zero if city/target point)
                if c_obs ~=0
                    other_from_c_obs = setdiff(find(node_to_obs==c_obs),[i,j]); %other nodes from obs of current node (remove i and j, if the same)
                    xP = Nodes(1,other_from_c_obs); yP = Nodes(2,other_from_c_obs); %vectors with points P to test
                    vect_sign = sign((xB - xA) * (yP - yA) - (yB - yA) * (xP - xA));%vector of signs
                    if sum(vect_sign==1)>0 && sum(vect_sign==-1)>0  %there are both signs, so not supporting or separating
                        sup_sep = 0;
                        break; %don't test for node j, if at i
                    end
                end
            end
            if sup_sep == 1 %supoprting or separating arc
                Adj(i,j) = norm(segm(:,1)-segm(:,2));   %Euclidean distance as cost
                Adj(j,i) = Adj(i,j);    %symmetrical adjacency matrix
                plot(segm(1,:),segm(2,:),'-g');
            end
        end
    end
end

end