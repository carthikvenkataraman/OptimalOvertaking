function updTask = UpdateScenario(task, decision)

% Update all vehicle states
updTask = task;

dt            = decision.ts(1);
updTask.E.y0  = decision.yE(2);
updTask.E.vx0 = decision.vEx(2);
updTask.E.vy0 = decision.vEy(2);

updTask.L.x0    = updTask.L.x0 + updTask.L.vx*dt;

if updTask.adjacentveh
    updTask.A.x0 = updTask.A.x0 + updTask.A.vx*dt;
end

if updTask.adjacentveh2
    updTask.A2.x0 = updTask.A2.x0 + updTask.A2.vx*dt;
end
end