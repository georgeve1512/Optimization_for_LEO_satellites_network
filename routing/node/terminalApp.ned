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

package node;

//
// TODO auto-generated module
//

simple TerminalApp
{
        parameters:
        int address;  // local node address
        int numOfTerminals;
        volatile double sendIaTime @unit(s) = default(exponential(1ms)); // time between generating packets
        volatile int packetLength @unit(byte) = default(1500byte);  // length of one message 1 megabit
        volatile double burst_next_interval @unit(s) = default(truncnormal(100s,20s));
        volatile double burst_next_event @unit(s) = default(truncnormal(10s,2s));
        volatile int burst_size  = default(truncnormal(300,50));
        @display("i=block/browser");
        @signal[endToEndDelay](type="simtime_t");
        @statistic[endToEndDelay](title="end-to-end delay of arrived packets";unit=s;record=vector,mean,max,count;interpolationmode=none);
        
    gates:
        input localIn;
        output localOut;    
}
