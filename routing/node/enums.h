#ifndef ENUM_LIST
#define ENUM_LIST
// Labels
enum lables{
    hot_potato_lable = -5,
};

// Signaling object (used to notify router module on current queue status)
enum last_routin {
    leader_to_roots = 21,
    route_from_root = 22,
    app_initial = 23,
    create_message = 24,
    message = 25,
    load_balance_reset = 26,
    //load balancer
    //TODO: might be buggy when number of nodes is greater than 25
    hot_potato = 27,
    ping_app = 28,
    ping_ping = 29,
    ping_pong = 30,
    load_prediction = 31,
    load_prediction_update = 32,
    update_topology_leader = 33,
    update_topology = 34,
    turn_off_link = 35,
    burst_app = 36,
    burst_flow = 37,
};

// Leader Election
enum phase_list {
    phase1_update = 11,                 //Every node keep updates the id table untill he learns all the nodes (given const number)
    phase2_link_info = 12,              //when finish the second phase, all the nodes wait for the topology  update to send packets
    phase3_building_LAST_tree = 13,     //Only the leader build the LAST_Tree and advertise it to all nodes
  //  phase4_root_broadcast = 14,       //Only last root's will be recieving this message to build LAST tree
};

// Directions
enum direction_list {
    left_d = 0,
    up_d = 1,
    right_d = 2,
    down_d = 3,
    diagonal_up = 4,
    diagonal_down = 5,
};

// Packet types
enum packet_type_list {
    initiale_packet = 1,
    regular_packet = 2,
    schedule_topo_packet = 3,
    send_node_info_packet = 4,
    update_app_packet = 10,
    terminal_list = 5,
    terminal_message = 6,
    terminal_connect = 7,
    terminal_disconnect = 8,
    terminal_index_assign = 9,
    terminal_list_resend = 15,
};

// Kind list - for future use
enum kind_list {
    legacy = 0,
    terminal = 1,
};

// Terminal/satellite modes
enum terminalModes{
    main,
    sub
};

// Terminal connection status
enum terminalConnectionStatus{
    disconnected = 0,
    connected = 1
};

// Types of self messages in terminal
enum selfMessageTypes{
    interArrivalTime,
    selfPositionUpdateTime,
    mainSatellitePositionUpdateTime,
    subSatellitePositionUpdateTime,
    initializeMyPosition,
    initializeConnection
};

#endif
