// This file is part of an OMNeT++/OMNEST simulation example. Copyright (C) 1992-2015 Andras Varga This file is distributed WITHOUT ANY WARRANTY. See the file.`license' for details on this and other legal matters.

package node;
import inet.mobility.contract.IMobility;
import inet.mobility.single.TurtleMobility;
import inet.mobility.static.StaticGridMobility;


//import inet.visualizer.integrated.IntegratedMultiVisualizer;
//
// A "Node" consists of a Routing plus an App module which represents the
// aggregate traffic of the subnets connected to that router.
//
module Node
{
    parameters:
        int address;
        int deltaValue = default(4);
        int Hops = default(4);
        int alpha;
        int load_threshold = default(90);
        int m_var;
        int frameCapacity = default(1000); // Queue Size :max number of packets; 0 means no limit
        int num_of_satellite = default(25);
        int changeRate @unit(s) = default(30s);
        bool load_balance_mode = default(true);
        bool load_balance_link_prediction = default(false);
        bool fullTopoRun = default(false);
        string appType;
        string mobilityType = default("SuperpositioningMobility");
        double recoveryFromFaultInterval @unit(s) = default(700s);	// The amount of time to recover from faults
        double timeToNextFault @unit(s) = default(exponential(10s));	// The time for the next fault 
        int methodToChooseRoots = default(1);	// 0 - Choose roots at random, 1 - Choose roots with Michael's algorithm
        int allowedNumberOfLinksDown = default(0); // How many links are allowed to be disabled in the entire simulation
        
        // Terminal handling
        int maxTerminalNumber = default(10);
        int maxHopCountForTerminalList = default(2);

        @networkNode();
        @signal[check](type="long");
        @statistic[coordX](title="coord X"; source="xCoord(mobilityPos(mobilityStateChanged))"; record=vector; interpolationmode=linear);
        @statistic[coordY](title="coord Y"; source="yCoord(mobilityPos(mobilityStateChanged))"; record=vector; interpolationmode=linear);
        @display("i=misc/satellite2,gold");
    gates:
        inout port[];
        input terminalIn[maxTerminalNumber] @directIn;
    submodules:
        app: <appType> like IApp {
            parameters:
                address = address;
                @display("p=140,42");
        }
        routing: Routing {
            parameters:
                @display("p=140,109");
            gates:
                in[sizeof(port)];
                out[sizeof(port)];
                terminalIn[maxTerminalNumber];
        }
        queue[sizeof(port)]: L2Queue {
            parameters:
                @display("p=65,155,row");
        }
        grid: StaticGridMobility {
            @display("p=45,233");
        }
        mobility: TurtleMobility {
            @display("p=116,233");
        }
    connections:
        routing.localOut --> app.in;
        routing.localIn <-- app.out;
        for i=0..sizeof(port)-1 {
            routing.out[i] --> queue[i].in;
            //from queue to routing (incoming)
            routing.in[i] <-- queue[i].out;
            //line is queue to queue
            queue[i].line <--> port[i];
        }
        for i=0..maxTerminalNumber-1{
            routing.terminalIn[i] <-- terminalIn[i];
        }
}
