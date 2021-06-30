result = load('result.txt');

figure,
plot(result(:,2),result(:,3),'ro')
hold on
plot(cpt_data.pos_ned(:,1),cpt_data.pos_ned(:,2),'g.')