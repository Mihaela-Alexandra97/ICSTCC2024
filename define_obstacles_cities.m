function [Obstacles,Cities,fig_handle] = define_obstacles_cities(env_bounds,n_obs,n_cities)
fig_handle = figure();
axis('equal');
axis(env_bounds);
box on
hold on

colors = ['k']; %if you want more random colors for obstacles, complete here

%% define obstacles and store the coordinates
%left click picks a vertex, right click (or a key) goes to next obstacle, without reading the current point
uiwait(msgbox(sprintf('\nFor defining an obstacle:\n\t - left-click = pick a vertex\n\t - right-click = pick last vertex\nObstacles should be convex and non-overlapping\n'),'Obstacles','modal'));
Obstacles = cell(1,n_obs);
for i=1:n_obs
    title(['Define obstacle #' num2str(i) ' from ' num2str(n_obs)]);

    counter = 1;    %number of vertices in current obstacle
    button = 1;
    Obstacles{i}.vertices = [];
    indx_color = randi(length(colors),1);   %random obstacle color
    Obstacles{i}.color = colors(indx_color);
    while button==1
        [x,y,button] = ginput(1);   %read one point
        if (x >= env_bounds(1) && x <= env_bounds(2) && y >= env_bounds(3) && y <= env_bounds(4)) %inside bounds
            plot(x,y,'.k');
            Obstacles{i}.vertices = [Obstacles{i}.vertices [x;y]];
            counter = counter + 1;
        end

        if button~=1    %try to finish current obstacle (button empty if Enter was pressed)
            try   %finish obstacle if can construct convex hull
                cv = convhull(Obstacles{i}.vertices(1,:),Obstacles{i}.vertices(2,:));
            catch %otherwise give a warning and continue to read vertices
                uiwait(msgbox(sprintf('A convex hull cannot be obtained from the set of points.\nContinue with curretn obstacle')));
                button = 1;
            end
        end
    end
    
    cv(end) = [];   %remove duplicated vertex (will close the polygon later
    Obstacles{i}.vertices = Obstacles{i}.vertices(:,cv);
    Obstacles{i}.handle = fill(Obstacles{i}.vertices(1,:),Obstacles{i}.vertices(2,:),Obstacles{i}.color);
   
    Obstacles{i}.center = [mean(Obstacles{i}.vertices(1,:)); mean(Obstacles{i}.vertices(2,:))];
    text(Obstacles{i}.center(1),Obstacles{i}.center(2),['O_{' num2str(i) '}'],'Color','w','FontWeight','bold','HorizontalAlignment','center');
end

%% define starting point and cities
%left or right click picks a point
title('Define cities');
uiwait(msgbox(sprintf('\nDefine starting point and cities with mouse clicks:\n\t-points should be in the free space\n\t-See fig''s title for current point'),'Cities','modal'));
Cities = NaN(2,n_cities); %matrix where each column contains the coordinates of a city.

for i=1:n_cities
    if i==1
        title('Define starting point');
    else
        title(['Define city #' num2str(i-1) ' from ' num2str(n_cities-1)]);
    end
    good_point = 0;
    while good_point == 0    %read point(s) until it is in free space
        [x,y] = ginput(1);   %read one point
        good_point = 1;  %assume good point
        if ~(x >= env_bounds(1) && x <= env_bounds(2) && y >= env_bounds(3) && y <= env_bounds(4)) %inside bounds
            good_point = 0;
            uiwait(msgbox(sprintf('\nWrong point - outside bounds.\nChoose again')));
        else %inside boundaries
            for j=1:n_obs
                if inpolygon(x,y,Obstacles{j}.vertices(1,[1:end 1]),Obstacles{j}.vertices(2,[1:end 1]))    %should close the polygon with first point as last
                    good_point = 0;
                    uiwait(msgbox(sprintf('\nWrong point - inside obstacle %d.\nChoose again',j)));
                    break;
                end
            end
            if i>=2 %test if it is a new point
                if ~isempty(intersect(Cities(:,1:i-1)',[x y],'rows'))
                    good_point = 0;
                    uiwait(msgbox(sprintf('\nWrong point - identical to one previously defined.\nChoose again')));
                end
            end
        end
        if good_point == 1   %good point
            if i==1
                plot(x,y,'ob'); %start point with blue circle
                text(x,y,'s','Color','k','FontWeight','bold','HorizontalAlignment','center','VerticalAlignment','top');
            else
                plot(x,y,'pr'); %cities with red star
                text(x,y,['c_{' num2str(i-1) '}'],'Color','k','FontWeight','bold','HorizontalAlignment','center','VerticalAlignment','top');
            end
            Cities(:,i) = [x;y];
        end
    end
end
title('');
end
