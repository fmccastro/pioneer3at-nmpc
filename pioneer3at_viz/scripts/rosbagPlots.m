%%  Plots

set(gca, 'defaultTextInterpreter', 'latex');
set(gca,'TickLabelInterpreter','latex');

p1 = plot(refX, refY, 'b');
hold on
p2 = plot(X, Y, 'r--');
legend([p1 p2], {'Desired Pose', 'True Pose'}, 'Interpreter', 'latex');
xlabel('X [m]', 'Interpreter', 'latex');
ylabel('Y [m]', 'Interpreter', 'latex');

distance = 0;
index = 1;
indexCheckPt = 1;

distPoseX = [];
distPoseY = [];
checkpoint = 10:10:100;

while( index < length( refX ) )
    distance = distance + sqrt( ( refX( index + 1 ) - refX( index ) )^2 +  ( refY( index + 1 ) - refY( index ) )^2 );
    
    if( distance > checkpoint(indexCheckPt) )
        distPoseX = [distPoseX, refX(index + 1)];
        distPoseY = [distPoseY, refY(index + 1)];
        indexCheckPt = indexCheckPt + 1;
    end
    
    index = index + 1;
end

disp( distance );
checkpoint = string( checkpoint );

figure

p1 = plot(refX, refY, 'b');
hold on
p2 = plot(distPoseX, distPoseY, 'r.', 'MarkerSize', 15);
hold on
text(distPoseX, distPoseY, checkpoint( 1:length(distPoseX) ), 'HorizontalAlignment', 'right', 'VerticalAlignment', 'bottom', 'FontSize', 15, 'FontWeight', 'bold', 'Interpreter', 'latex');
legend([p1 p2], {'Desired Pose', 'Distance[m]'}, 'Interpreter', 'latex');
xlabel('X [m]', 'Interpreter', 'latex');
ylabel('Y [m]', 'Interpreter', 'latex');