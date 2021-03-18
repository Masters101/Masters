 %Draw Support System

%%%%%%%%%%%%%%%% Floor Coordinates %%%%%%%%%%%%%%%%%%%%%%%
floor_x = [-2 -2 2 2 -2];
floor_y = [-0.01 0 0 -0.01 -0.01];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%% Wheel Coordinates %%%%%%%%%%%%%%%%%%%%%%
th = 0:pi/100:2*pi;
a = r*cos(th);
b = r*sin(th);
offset1 = 0.35;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%% Cart Body %%%%%%%%%%%%%%%%%%%%%%%%
cart_x = [0 0 1 1 0];
cart_y = [0 0.5 0.5 0 0];
offset2 = 0.5;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%     

%%%%%%%%%%%%%%%%%%%%%% Boom %%%%%%%%%%%%%%%%%%%%%%%%%%%%
boom_x = [0 0 0.2 0.2 0];
boom_y = [0.75 0.95 0.95 0.75 0.75];
offset3 = 0.1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for i = 1:length(out.Output)
    % Plot Cart and wheels
    plot(offset1+out.Output(i)+a,r+b,'b.') % Wheel 1
    hold on
    plot(-offset1+out.Output(i)+a,r+b,'b.') % Wheel 2
    plot(floor_x,floor_y,'k-') % Floor
    plot(cart_x+out.Output(i)-offset2,cart_y +(2.3*r),'r-','LineWidth',2)
    
    % Plot Boom 
    plot(boom_x+out.Input(i)-offset3,boom_y,'g-','LineWidth',2)
    
    hold off
    axis([-2 2 -1 5])
    pause(0.01)
end
 
