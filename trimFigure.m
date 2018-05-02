function trimFigure(ax)

outerpos = ax.OuterPosition;
ti = ax.TightInset; 
ti(1) = ti(1) + 0.1;
ti(3) = ti(3) + 0.1;
left = outerpos(1) + ti(1);
bottom = outerpos(2) + ti(2);
ax_width = outerpos(3) - ti(1) - ti(3);
ax_height = outerpos(4) - ti(2) - ti(4);
ax.Position = [left bottom ax_width ax_height];

end