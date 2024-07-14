clear all;
close all;
clc;
%%
env_bounds = [0 30 0 20];
n_obs = 3;
n_cities = 4;  %excluding starting point
n_cities = n_cities+1;  %first city will be starting point
e=[0 1 2 4 5 ]'
l=[10 7 4 7 10]'
v=5.5;
%%
[Obstacles,Cities,init_fig_handle] = define_obstacles_cities(env_bounds,n_obs,n_cities);
for i=1:n_cities-1
    for j=i+1:n_cities
        t(i,j) = norm(Cities(:,i)-Cities(:,j))/v;
        t(j,i) = t(i,j);
    end
end
%Plot TW n obstacle vertices com
for i=1:n_cities
    text(Cities(1,i),Cities(2,i)-1.15,['[' num2str(e(i)),', ',num2str(l(i)) ']'],'Color','k','FontWeight','normal','HorizontalAlignment','center','VerticalAlignment','top','FontSize',15);
end
for i=1:length(Obstacles)
    
%     for j=1:size(Obstacles{i}.vertices,2)
        for k=1:size(Obstacles{i}.vertices,2)
        text(Obstacles{i}.vertices(1,k)-0.3,Obstacles{i}.vertices(2,k)+1,['v_{' num2str(k),',', num2str(i) '}'],'Color','m','FontWeight','bold','HorizontalAlignment','center','VerticalAlignment','top','FontSize',10);
        end
%     end
end
%%
[Nodes_coord,Adj_full] = construct_visibility_graph(Obstacles,Cities,init_fig_handle,v);
%%
[Adj_red,Edge_to_path,ind_red_to_full] = reduce_graph(Adj_full,1:n_cities); %reduced graph with first n_cities
costs=Adj_red;  
%%

       



%%
%the returned path_cost should be equal with the cost of the sequence from reduced graph
M=1000;
VRPTW=optimproblem;

x = optimvar('x_ij',n_cities,n_cities,'Type','integer','LowerBound',0,'UpperBound',1); %xij=1 if (i,j) is used by the vehicle
d=optimvar('Di',n_cities,'Type','continuous','LowerBound',0,'UpperBound',max(l)); % departure time at customer i
VRPTW.Objective =v* sum(sum(costs.*x)); %optimization function (eq1)

%Define constraints
constr_enter = optimconstr(n_cities);   %enter each city one time
for i=1:n_cities
     j=setdiff(1:n_cities,i);
    constr_enter(i) = (sum(x(i,j)) == 1);
end
VRPTW.Constraints.EnterConstraints = constr_enter; %first set of constraints(eq2)

constr_leave = optimconstr(n_cities);   %leave each city one time
for j=1:n_cities
    i=setdiff(1:n_cities,j);
    constr_leave(j) = (sum(x(i,j)) == 1);
end
VRPTW.Constraints.constraints2 = constr_leave;    %second set of constraints

constr_travel_time=optimconstr;
k=1;
for i=2:n_cities-1
    for j=setdiff(2:n_cities-1,i)
    constr_travel_time(k) = ((d(i)+t(i,j)-d(j))<=((1-x(i,j) )*M));
    k=k+1;
    end
end
VRPTW.Constraints.TravelTimeConstraints=constr_travel_time;% third set of constraints(eq4A)

constr_time1=optimconstr(n_cities);
for i=1:n_cities
    constr_time1(i)=(e(i)<=d(i));
end
VRPTW.Constraints.TWConstraints1 = constr_time1;%fourth set of constraints(eq5)

constr_time2=optimconstr(n_cities);
for i=1:n_cities
    constr_time2(i)=(d(i)<= l(i));
end
VRPTW.Constraints.TWConstraints2 = constr_time2;%fourth set of constraints(eq5)

% % fifth set of constraints (eq7) is in optimvar
%subtour elimination
constr_tour_elim=optimconstr;
for i=2:n_cities
    i
    for j=setdiff(2:n_cities,i)
        j
        constr_tour_elim(end+1) = ((d(i)-d(j)+(n_cities)*x(i,j))<=(n_cities-1));

    end
end
%modification added , counting starting at 2
VRPTW.Constraints.TourElimination =  constr_tour_elim;%sixth set of constraints(eq7)
%Solve optimization
constr_timp=optimconstr;
j=1;
for i=2:n_cities-1
   constr_timp(end+1) = ((d(i)+t(i,j)-d(j))<=((1-x(i,j) )*M));
end
VRPTW.Constraints.TravelTimeConstraints2= constr_timp;

opts = optimoptions('intlinprog','Display','off');

[solution, f_val, exitflag] = solve(VRPTW,'options',opts);
if exitflag ~= 1
    disp('ERROR')
else

    x_ij = round(solution.x_ij);
    no_trans = sum(sum(x_ij));  %number of transitions (should be n)
    i = 1;    %start from node 1
    path_red=1;  %***path_red is the obtained sequence through cities
    %*** node 1 is depot (initial)
    for trans=1:no_trans    %***am schimbat for-ul 182-192 - doar pt. aflare traseu, path_red ***
        j = find(x_ij(i,:)==1);    %i is the current node, j is the next one
        path_red(end+1)=j;
        i=j;
    end
end

[path_indices,path_coord,path_cost] = path_in_full(path_red,Edge_to_path,Nodes_coord);
arrival=[]
departure=[]
for i=2:n_cities
    for j=2:n_cities
  arrival(j-1)=solution.Di(i)-t(i,j);
  departure(i-1)=solution.Di(i);
    end
end
arrival
departure