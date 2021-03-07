//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

package networks;

import node.Node;
import ned.DatarateChannel;
import node.Terminal;


network NetK
{
    parameters:
        int num_of_hosts = default(25);
        int num_of_terminals = default(20);
		double maxX @unit(m);
        double maxY @unit(m);
        double minX @unit(m);
        double minY @unit(m);
        int colNum=default(int(ceil(sqrt(num_of_hosts * (maxX - minX) / (maxY - minY)))));
        
    	//@display("bgi=maps/world;bgb=763,318");
        //@display("bgi=background/earth(1);bgb=2000,1000");
    types:
        channel C extends DatarateChannel
        {
            parameters:
                delay = default(0ms);
                datarate = default(0.5Gbps);
        }
    submodules:
        rte[num_of_hosts]: Node {
            parameters:
                address = index;
                num_of_satellite = num_of_hosts;
                
                // Grid data + constraints
                grid.numHosts = num_of_hosts;
                grid.constraintAreaMaxX = maxX;
                grid.constraintAreaMaxY = maxY;
                grid.constraintAreaMaxZ = 0;
                grid.constraintAreaMinX = minX;
                grid.constraintAreaMinY = minY;
                grid.constraintAreaMinZ = 0;
                
                // Mobility constraints
                mobility.constraintAreaMaxX = maxX;
                mobility.constraintAreaMaxY = maxY;
                mobility.constraintAreaMaxZ = 0;
                mobility.constraintAreaMinX = minX;
                mobility.constraintAreaMinY = minY;
                mobility.constraintAreaMinZ = 0;
                
                @display("bgb=$maxX-$minX,$maxY-$minY;bgi=maps/world,s,bgg=100,2,grey95,bgu=km");
                //@display("i=device/satellite,white;p=,,m,$colNum,10000,10000"); 	// Matrix form
                //@display("i=device/satellite,white;p=100,100,ri,1000,1000");	// Ring form

        }
        terminal[num_of_terminals]: Terminal {
            parameters:
                address = index;
                num_of_terminals = num_of_terminals;
                
                // Mobility constraints
                mobility.constraintAreaMaxX = maxX - terminal[index].radius;
                mobility.constraintAreaMaxY = maxY - terminal[index].radius;
                mobility.constraintAreaMaxZ = 0;
                mobility.constraintAreaMinX = minX + terminal[index].radius;
                mobility.constraintAreaMinY = minY + terminal[index].radius;
                mobility.constraintAreaMinZ = 0;
                //mobility.initialX = fmod(normal((minX+maxX)/2,(maxX-minX)/7),maxX-minX);
                //mobility.initialY = fmod(normal((minY+maxY)/2,(maxY-minY)/7),maxY-minY);
                //mobility.initFromDisplayString = false;            
        }
    connections allowunconnected:
        // Connection map (for num_of_hosts=25, colNum=5):
        // 00 01 02 03 04
        // 05 06 07 08 09
        // 10 11 12 13 14
        // 15 16 17 18 19
        // 20 21 22 23 24

		// NOTE: The next connections (in loops) are inspired by Net25, and are to replicate it when
		//		 num_of_hosts=25, colNum=5. The build is EXACTLY the same build in Net25, but expanded
		//		 for any [num_of_hosts] or [colNum]

        // Connections to right neighbor and to the neighbor below (if exists): x -> y and x -> z
        //		... x y ...
        //		... z . ...
        for j=0..int(num_of_hosts/colNum)-1, for i=0..colNum-1 {
            rte[colNum*j+i].port++ <--> C <--> rte[colNum*j+i+1].port++ if i!=colNum-1;						// x -> y, w/o last col
            rte[colNum*j+i].port++ <--> C <--> rte[colNum*(j+1)+i].port++ if j!=int(num_of_hosts/colNum)-1;	// x -> z, w/o last row
        }

        // Torus
        // Completing outer circles: x -> z or x -> y
        // NOTE: x,y,z are in margins
        //		x	...	 y
        //		...
        //		z
        for j=0..int(num_of_hosts/colNum)-1 {
            rte[colNum*j].port++ <--> C <--> rte[colNum*(j+1)-1].port++;									// x -> y
        }

        for j=0..colNum-1 {
            rte[colNum*(int(num_of_hosts/colNum)-1) + j].port++ <--> C <--> rte[j].port++;					// x -> z
        }

    	// Fifth Link
    	// Diagonal connections: x -> y -> z
    	//		x . .
    	//		. y .
    	//		. . z
    	// NOTE: Odd ruling for legacy porpuses

        // y -> first -> last
        rte[0].port++ <--> C <--> rte[colNum+1].port++;														// first -> y
        rte[0].port++ <--> C <--> rte[num_of_hosts-1].port++;												// first -> last 

        for j=1..colNum-2 {
            rte[j].port++ <--> C <--> rte[colNum+1+j].port++;												// x -> y
            rte[j].port++ <--> C <--> rte[num_of_hosts-colNum-1+j].port++;									// x -> z
        }

        rte[colNum-1].port++ <--> C <--> rte[num_of_hosts-2].port++;										// z -> y
        rte[colNum-1].port++ <--> C <--> rte[colNum].port++;   												// z -> x
        for i=1..int(num_of_hosts/colNum)-2, for j=colNum*i..colNum*(i+1)-1 {
            rte[j].port++ <--> C <--> rte[j+colNum+1].port++ if j!=colNum*(i+1)-1;							// x -> y, w/o last col 
            rte[j].port++ <--> C <--> rte[j+1].port++ if j==colNum*(i+1)-1;									// x -> y, last col
        }
}
