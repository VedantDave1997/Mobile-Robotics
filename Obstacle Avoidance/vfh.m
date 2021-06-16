function steeringdirection = vfh(ranges, angles, targetdir, varargin)
    
    plotGraphs = varargin{1};
    %ranges = ranges1; angles = angles1_t; 
    numsectors = 128; rmax=5; alpha=1.5;narrow=0.12;
    %rmax = 5; alpha = 2.8;narrow =0.12;numsectors = 64; %hmax=3.8;
    
    ranges = max(0,ranges);
    minangle = min(angles);
    maxangle = max(angles);
    sectorincrement = (maxangle - minangle)/numsectors;
    sectormidpoints = linspace(minangle, maxangle, numsectors);
    sector_edges = [sectormidpoints - sectorincrement/2, sectormidpoints(end) + sectorincrement/2];

    valididx = find(ranges<rmax);
    validranges = ranges(valididx);
    validangles=angles(valididx);

    weightedRanges = (1 - validranges/rmax).^alpha;
    hmax = max(weightedRanges);
    [~, edges, bin] = histcounts(validangles, sector_edges);
    obstacleDensity = zeros(1, numsectors);


    for i=1:length(bin)
        obstacleDensity(1, bin(i)) = obstacleDensity(1, bin(i)) + weightedRanges(i);
    end
    %b = [1 2 3 4 3 2 1];
    %obstacleDensity = filtfilt(b/7,1,obstacleDensity);
    
    sectoroccupied = zeros(1, numsectors); %boolean
    sectoroccupied(obstacleDensity > hmax) = true;
    
    changes = diff([0 ~sectoroccupied 0]);
    foundSectors = find(changes);
    sectors = reshape(foundSectors, 2, []);
    sectors(2,1:end) = sectors(2,1:end) - ones(1, size(sectors,2));
    clear_path = true;
    steeringdirection = NaN;
    numberSolns =  size(sectors);
    
    for i=1:numberSolns(2)
        if targetdir > sectormidpoints(sectors(1,i)) && targetdir < sectormidpoints(sectors(2,i))
            steeringdirection = targetdir;
        elseif(i == length(sectors))
            clear_path = false;
        end
    end
    wide_valley = [];
    narrow_valley = [];    
    if clear_path == false
        
        for i=1:length(sectors)
            if sectormidpoints(sectors(2,i)) - sectormidpoints(sectors(1,i)) > narrow
                wide_valley = [wide_valley , sectormidpoints(sectors(1,i))+narrow/2, ...
                    sectormidpoints(sectors(2,i)) - narrow/2];
            else
                narrow_valley = [narrow_valley , (sectormidpoints(sectors(2,i))+sectormidpoints(sectors(1,i)))/2 ];
            end
        end
        if isempty(wide_valley) && isempty(narrow_valley)
            steeringdirection = NaN;
        elseif min(abs(narrow_valley - targetdir)) < min(abs(wide_valley - targetdir))
            closest_target_dir = min(abs(narrow_valley - targetdir));
            steeringdirection = narrow_valley(abs(narrow_valley - targetdir) == closest_target_dir);
        else
            closest_target_dir = min(abs(wide_valley - targetdir));
            steeringdirection = wide_valley(abs(wide_valley - targetdir) == closest_target_dir);
        end
    end
    
    if plotGraphs
        subplot(2,2,1);
        polarplot(angles,ranges,'r');
        title('Laser Scan');
        subplot(2,2,2);
        polarplot(sectormidpoints,obstacleDensity);
        title('Polar Density');
        subplot(2,2,3);
        polarplot(sectormidpoints,sectoroccupied);
        title('Binary Occupancy Histogram');
        subplot(2,2,4);
        polarplot([0 targetdir],[0 1],'r');
        hold on
        polarplot( sectormidpoints,sectoroccupied, 'y');
        hold on
        if ~isnan(steeringdirection)
            polarplot([0 steeringdirection],[0 1],'b');
        end
        title('Steering Direction');
        legend('Target direction','Occupancy','Steering Direction');
    end
end

