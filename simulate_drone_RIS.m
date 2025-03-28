function simulate_drone_RIS()
    clc;
    num_tx = 5; % 1RIS | 3RIS | 5RIS | 7RIS
    track_type = 4; % 1-linear | 2-zig_zag | 3-random | 4-same_random
    % speed_values = [0.05, 0.02, 0.01, 0.005]; % 10000, 25000, 50000, 100000
    speed_values = [0.005, 1]; 

    for j = 1:1
        speed = speed_values(j);
 
        track_length = 500;  
        center_frequency = 10.7e9;  
    
        simpar = qd_simulation_parameters;
        simpar.center_frequency = center_frequency;
        simpar.sample_density = 5;
        
        % calcola la frequenza di campionamento e il numero di campioni per metro
        fs = 2 * simpar.center_frequency * (simpar.sample_density / simpar.speed_of_light);
        simpar.samples_per_meter = fs / speed;
        sample_rate = 1; %1m
        sample_per_sec = 2;
     
        rx = qd_arrayant('omni');      
        rx.center_frequency = center_frequency;
    
        tx1 = qd_arrayant('omni');  tx1.center_frequency = center_frequency;
        switch num_tx
            case 3
                tx2 = tx1.copy;
                tx3 = tx1.copy;
            case 5
                tx2 = tx1.copy;
                tx3 = tx1.copy;
                tx4 = tx1.copy;
                tx5 = tx1.copy;
            case 7
                tx2 = tx1.copy;
                tx3 = tx1.copy;
                tx4 = tx1.copy;
                tx5 = tx1.copy;
                tx6 = tx1.copy;
                tx7 = tx1.copy;
        end    
    
        h_layout = qd_layout(simpar);
        h_layout.update_rate = get_update_rate(sample_per_sec);
    
        h_layout.no_rx = 1;
        h_layout.rx_array = rx;
        z = randi([1, 5]);
        initial_pos = [2;10;2];     % 𝐏𝐎𝐒𝐈𝐙𝐈𝐎𝐍𝐄 𝐈𝐍𝐈𝐙𝐈𝐀𝐋𝐄
        h_layout.rx_position = initial_pos;
        
        h_layout.no_tx = num_tx;
        % Linear, Zig:[track_length*0.9;2;3], [track_length/2;track_length/2;1]
        % Random:[track_length*0.6;2;3], [track_length/4;track_length/2;1]
        switch num_tx
            case 1
               h_layout.tx_array = tx1;
               h_layout.tx_position = [track_length/2;track_length*0.3;2];
            case 3
               h_layout.tx_array = [tx1, tx2, tx3];
               h_layout.tx_position = [[0;0;2], [track_length*0.6;2;3], [track_length/4;track_length/2;1]];
            case 5
               h_layout.tx_array = [tx1, tx2, tx3, tx4, tx5];
               h_layout.tx_position = [[0;0;2], [track_length*0.6;2;3], [track_length/4;track_length/2;1], [track_length*0.9; track_length*0.3; 4],[track_length*0.3; track_length*0.9; 2]];
            case 7
               h_layout.tx_array = [tx1, tx2, tx3, tx4, tx5, tx6, tx7];
               h_layout.tx_position = [[0;0;2], [track_length*0.6;2;3], [track_length/4;track_length/2;1], [track_length*0.9; track_length*0.3; 4],...
                   [track_length*0.3; track_length*0.9; 2],[track_length/4;track_length*0.3;5],[track_length*0.7;track_length*0.8;1]];
        end
        
        switch track_type
                case 1  % track rettilinea
                    h_layout.rx_track = generate_linear_track('DroneLinear', track_length, 0, initial_pos, speed, sample_rate); 
                    name = 'linear';
                case 2  % track zig-zag
                    h_layout.rx_track = generate_zigzag_track('DroneZigzag', track_length, 0, initial_pos, speed, sample_rate);
                    name = 'zig';
                case 3  % track randomica
                    h_layout.rx_track = generate_random_track('DroneRandom', track_length, 0, initial_pos, speed, sample_rate); 
                    name = 'random';
                case 4 
                    h_layout.rx_track = generate_same_random_track('DroneRandom', track_length, 0, initial_pos ,speed, sample_rate);
                    name = 'random';
        end
    
        % associa il ricevitore a ciascun trasmettitore
        % primo membro tx secondo mebro rx
        switch num_tx
            case 1
               h_layout.pairing = [1;1];
            case 3
               h_layout.pairing = [[1;1],[2;1],[3;1]];
            case 5
               h_layout.pairing = [[1;1],[2;1],[3;1],[4;1],[5;1]];
            case 7
               h_layout.pairing = [[1;1],[2;1],[3;1],[4;1],[5;1],[6;1],[7;1]];
        end    
    
        % "1" -> tempo iniziale, "0.5"-> durata/step, 'linear'-> metodo di interpolazione, 'false' disabilita opzioni extra
        [~, h_builder] = h_layout.get_channels(0, 0, 'linear', false);
    
        dataset = generate_dataset(h_builder, h_layout, initial_pos, num_tx);
    
        noisy_dataset = generate_noise(dataset); 
        dataset_with_loss = remove_random_rows(noisy_dataset, num_tx);  % rimuove il 10% delle righe
    
        nomeUtente = getenv('USERNAME');
        directory = ['C:\Users\' nomeUtente '\Desktop\dataset\1_drone\without_loss\' num2str(num_tx) 'RIS\Dataset_noisy_' name];
        directory_loss = ['C:\Users\' nomeUtente '\Desktop\dataset\1_drone\with_loss\' num2str(num_tx) 'RIS\Dataset_noisy_' name];
        nomeFile = ['Dataset_noisy_' name '_' num2str(h_layout.rx_track.no_segments) '.mat'];
        fullPath = fullfile(directory, nomeFile);
        fullPath_loss = fullfile(directory_loss, nomeFile);
    
        if exist(fullPath, 'file') 
            error('Il file %s esiste già. Operazione annullata per evitare sovrascrittura.', fullPath);
        else
            save(fullPath, 'noisy_dataset');
        end
        if exist(fullPath_loss, 'file') 
            error('Il file %s esiste già. Operazione annullata per evitare sovrascrittura.', fullPath_loss);
        else
            save(fullPath_loss, 'dataset_with_loss');
        end
    
        disp(['Dimensione dataset: ', mat2str(size(dataset))]); 
        disp(['Dimensione noisy_dataset: ', mat2str(size(noisy_dataset))]); 
        disp(['Dimensione dataset_loss: ', mat2str(size(dataset_with_loss))]); 
    
        % print(h_layout);
        % print_dataset(dataset);
    end
    disp('debug');

end

function update_rate = get_update_rate(sample_per_sec)
    update_rate = 1/sample_per_sec;
end

function new_track = generate_linear_track(name, track_length, orientation, init_pos, speed, sample_rate)
    new_track = qd_track('linear', track_length, orientation);
    new_track.name = name;
    new_track.initial_position = init_pos;
    new_track.set_speed(speed);

    [~, new_track] = new_track.interpolate('time', sample_rate, new_track.movement_profile, 'linear', false);
    
    disp(['lunghezza track: ', num2str(new_track.get_length)]);

    addSegment(new_track, init_pos);
end

function new_track = generate_zigzag_track(name, track_length, orientation, init_pos, speed, sample_rate)
    new_track = qd_track('linear', track_length, orientation);
    new_track.name = name;
    new_track.initial_position = init_pos;
    new_track.set_speed(speed);

    x=1;
    n=5; % deve essere un numero divisibile per track_length
    dist = track_length/n;
    positions = [0;0;0];  
    
    for i = 1:n
        x=x+1;
        if rem(x,2)==0
            new_x = positions(1,end) + dist;
            new_y = positions(2,end) + dist*sin(45*pi*180); 
            new_z = positions(3,end); 
        else
            direction = 45 * (pi / 180); 
            new_x = positions(1,end) + dist * cos(direction);
            new_y = positions(2,end) + dist * sin(direction);
            new_z = positions(3,end);
        end
        positions = [positions, [new_x; new_y; new_z]];
    end
    new_track.positions = positions;
    new_track.no_snapshots = size(positions, 2);

    [~, new_track] = new_track.interpolate('time', sample_rate, new_track.movement_profile, 'linear', false);

    disp(['lunghezza track: ', num2str(new_track.get_length)]);

    addSegment(new_track, init_pos);
end

function new_track = generate_random_track(name, track_length, orientation, init_pos, speed, sample_rate)
    new_track = qd_track('linear', track_length, orientation);
    new_track.name = name;
    new_track.initial_position = init_pos;
    new_track.set_speed(speed);

    total_distance = 0;
    distanza = 0;
    x=0;
    positions = [0;0;0];  
    
    while total_distance < track_length
        move_type = randi([1, 2]); 

        % if per far si che non ci siano 2 scambi direzionali consegutivi
        if (move_type == 2 && x == 1)
            move_type = 1;
            x = 0;
        end   

        switch move_type
            case 1  % linea
                dist = randi([1, 2])*20; 
                new_x = positions(1,end) + dist;
                new_y = positions(2,end) + dist*sin(45*pi*180); % moltiplico per sen(45) per avere la direzione parallale all'asse x
                new_z = positions(3,end);  
            case 2  % cambio direzionale
                x=1; 
                dist = randi([1, 2])*30; %30+20=50
                direction = randi([15, 135]) * (pi / 180); 
                new_x = positions(1,end) + dist * cos(direction);
                new_y = positions(2,end) + dist * sin(direction);
                new_z = positions(3,end);
        end
        distanza = total_distance;
        total_distance = total_distance + dist;
        % controllo se con l'ultimo segmento ho sforato la track_length e
        if(total_distance > track_length)
            dif = track_length - distanza;
            new_x = positions(1,end) + dif;
            new_y = positions(2,end) + dif*sin(45*pi*180); % moltiplico per sen(45) per avere la direzione parallale all'asse x
            new_z = positions(3,end);
        end    
        positions = [positions, [new_x; new_y; new_z]];
    end
    new_track.positions = positions;
    new_track.no_snapshots = size(positions, 2);
    disp(['lunghezza track prima di interpolare: ', num2str(new_track.get_length)]);

    nomeUtente = getenv('USERNAME');
    % save(['C:\Users\' nomeUtente '\Desktop\dataset\Random_positions.mat'], 'positions');

    [~, new_track] = new_track.interpolate('time', sample_rate, new_track.movement_profile, 'linear', false);

    disp(['lunghezza track dopo aver interpolato: ', num2str(new_track.get_length)]);

    addSegment(new_track, init_pos);
end

function new_track = generate_same_random_track(name, track_length, orientation, init_pos, speed, sample_rate)
    new_track = qd_track('linear', track_length, orientation);
    new_track.name = name;
    new_track.initial_position = init_pos;
    new_track.set_speed(speed);

    nomeUtente = getenv('USERNAME');
    load(['C:\Users\' nomeUtente '\Desktop\dataset\random_positions\Random_positions.mat'], 'positions');
    % load(['/Users/' nomeUtente '/Desktop/dataset/random_positions/Random_positions.mat'], 'positions');
    new_track.positions = positions;
    new_track.no_snapshots = size(positions, 2);

    [~, new_track] = new_track.interpolate('time', sample_rate, new_track.movement_profile, 'linear', false);
    disp(['lunghezza track dopo aver interpolato: ', num2str(new_track.get_length)]);

    addSegment(new_track, init_pos);
end

function addSegment(new_track, init_pos)
    scenario = {'3GPP_38.901_Indoor_LOS'}; %scenario decide numero di cluster (esempio 15)
    num_positions = size(new_track.positions, 2);
    abs_pos = zeros(3,size(new_track.positions, 2));
    for i = 1:size(abs_pos, 2)
        abs_pos(:, i) = new_track.positions(:, i) + init_pos;
    end
    for i = 1:(num_positions)  % i = 1:2:(num_positions)
        new_track.add_segment(abs_pos(:, i), scenario, []);
    end

    disp(['numero segmenti: ', num2str(new_track.no_segments)]);
end

function final = generate_dataset(h_builder, h_layout, initial_pos, n)
    new_track = h_layout.rx_track;
    abs_pos = zeros(3,size(new_track.positions, 2));
    for i = 1:size(abs_pos, 2)
        abs_pos(:, i) = initial_pos + new_track.positions(:, i);
    end
    pos = abs_pos(:, new_track.segment_index);
    pos_rip = repmat(pos, 1, n); % n numero tx
    disp(['numero di righe dataset: ',num2str(n),'*',num2str(length(pos)),'=', num2str(length(pos_rip))]);

    dataset = [];
    % Per ciascun trasmettitore (h_builder è 1x3)
    for i = 1:length(h_builder)
        % Estrai solo la prima colonna degli scatter (7x1)
        rssi = h_builder(i).pow(:,1);   
        aod  = h_builder(i).AoD(:,1);    
        aoa  = h_builder(i).AoA(:,1);   
        delay = h_builder(i).taus(:,1);
        
        % Trasforma ciascuna matrice in un vettore colonna (7x1)
        rssi_vec = rssi(:);
        aod_vec  = aod(:);
        aoa_vec  = aoa(:);
        delay_vec = delay(:);
        
        % Costruisci la matrice per questo trasmettitore (7x4)
        dataset_i = [rssi_vec, aod_vec, aoa_vec, delay_vec];
        
        % Accumula nel dataset totale
        dataset = [dataset; dataset_i];
    end
    % disp(size(dataset));
    final = [ num2cell(dataset) , mat2cell(pos_rip, size(pos_rip,1), ones(1,size(pos_rip,2)))' ];
end

function noisy_dataset = generate_noise(dataset)
    noise_std_rssi = 0.05;        
    noise_std_aod = 0.1;         
    noise_std_aoa = 0.1;         
    noise_std_delay = 1e-9;
    
    noisy_dataset = dataset;
    for i = 1:size(dataset, 1)
        noisy_dataset{i, 1} = dataset{i, 1} + noise_std_rssi * randn(); % RSSI
        noisy_dataset{i, 2} = dataset{i, 2} + noise_std_aod * randn();  % AoD
        noisy_dataset{i, 3} = dataset{i, 3} + noise_std_aoa * randn();  % AoA
        noisy_dataset{i, 4} = dataset{i, 4} + noise_std_delay * randn(); % Delay
    end
end

function dataset_filtered = remove_random_rows(dataset, num_tx)
    % Determina il numero di righe totali e il numero di righe iniziali da analizzare
    total_rows = size(dataset, 1);
    rows_to_check = total_rows / num_tx;
    disp(['Posizioni totali: ', num2str(rows_to_check)]);

    
    % Determina il 10% delle righe da rimuovere
    num_to_remove = round(0.1 * rows_to_check); % 10%
    disp(['Posizioni perse: ', num2str(num_to_remove)]);

    % Seleziona randomicamente le righe da eliminare tra le prime 'rows_to_check' righe
    rng('shuffle'); % Mescola il generatore di numeri casuali per risultati diversi ad ogni esecuzione
    remove_indices = randperm(rows_to_check, num_to_remove); 
    
    % Estrai le posizioni corrispondenti alle righe scelte
    removed_positions = dataset(remove_indices, 5); % Rimane in formato cell
    
    % Trova tutte le righe che contengono le stesse posizioni da eliminare
    rows_to_delete = false(total_rows, 1);
    
    for i = 1:length(removed_positions)
        position = removed_positions{i}; % Estraggo la posizione in formato cell
        
        % Crea una maschera per trovare le righe con la stessa posizione
        matching_rows = cellfun(@(x) isequal(x, position), dataset(:, 5));
        rows_to_delete = rows_to_delete | matching_rows;
    end
    
    % Rimuove le righe selezionate
    dataset_filtered = dataset(~rows_to_delete, :);
end

function print(h_layout)
    h_layout.visualize([],[],0);
    view(gca, -33,60);
    hold on;

    new_track = h_layout.rx_track;
    init_pos = new_track.initial_position;
    seg_positions = new_track.positions(:, new_track.segment_index) + init_pos;
    % plot3(seg_positions(1,:), seg_positions(2,:), seg_positions(3,:), 'go', 'LineWidth', 2, 'MarkerSize', 8, 'DisplayName', 'Posizione Segmenti');
    % plot3( h_layout.rx_track.positions(1,:) + init_pos(1), h_layout.rx_track.positions(2,:) + init_pos(2), h_layout.rx_track.positions(3,:) + init_pos(3), 'mo', 'LineWidth', 1, 'DisplayName', 'Percorso Effettuato');

    title('Simulazione Drone - Punti Inizio Segmenti');
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    grid on;
    hold off;
end

function print_dataset(dataset)
    disp('Dataset:');
    for i = 1:size(dataset,1)
        rssi = dataset{i,1}; 
        aod  = dataset{i,2}; 
        aoa  = dataset{i,3}; 
        delay = dataset{i,4}; 
        pos = dataset{i,5}; 
    
        fprintf('Riga %d: RSSI=%.2f, AoD=%.2f, AoA=%.2f, Delay=%.2f, Posizione=[%.2f, %.2f, %.2f]\n', ...
            i, rssi, aod, aoa, delay, pos(1), pos(2), pos(3));
    end
end
