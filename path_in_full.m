function [path_indices,path_coord,path_cost] = path_in_full(path_red,Edge_to_path,Nodes_coord)
%Convert path in reduced graph to path in full one
% "path_red" refers to reduced graph
% indices of "Edge_to_path" are from reduced graph, and returned elements of Edge_to_path{i,j} from full one
% ind_red_to_full,Nodes_coord and the output arguments refer to the full one

path_indices = [];
for i=1:length(path_red)-1
    path = Edge_to_path{path_red(i),path_red(i+1)};
    if i>1
        path(1) = [];   %already stored (end of previous path)
    end
    path_indices = [path_indices, path];    %indices of traversed nodes in full (initial) graph
end

path_coord = zeros(2,length(path_indices));
for i=1:length(path_indices)
    path_coord(:,i) = Nodes_coord(:,path_indices(i));   %x,y coordinates of traversed nodes
end

path_cost=0;
for i=1:size(path_coord,2)-1
    path_cost = path_cost + norm(path_coord(:,i)-path_coord(:,i+1));    %distance to travel

    %just for visualizing the path in visibility graph
    plot(path_coord(1,[i,i+1]),path_coord(2,[i,i+1]),'Color','r','LineWidth',2);
    pause(0.5);
end

end
