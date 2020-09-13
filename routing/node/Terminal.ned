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
import inet.mobility.contract.IMobility;

module Terminal
{
    parameters:
        int address;
        int hops = default(2);
        int num_of_terminals = default(2);
        double radius @unit(m) = default(1000m);
        double frac;
        string mobilityType;
        
        @networkNode();
        @signal[check](type="long");
        @statistic[coordX](title="coord X"; source="xCoord(mobilityPos(mobilityStateChanged))"; record=vector; interpolationmode=linear);
        @statistic[coordY](title="coord Y"; source="yCoord(mobilityPos(mobilityStateChanged))"; record=vector; interpolationmode=linear);
        @display("i=device/antennatower;bgb=129,100;r=$radius");
    gates:
        input mainIn;
        output mainOut;
        input subIn;
        output subOut;
    submodules:
        app: TerminalApp {
            parameters:
                address = address;
                numOfTerminals = num_of_terminals;
                @display("p=37,44");
        }
        mobility: <mobilityType> like IMobility {
            @display("p=89,44");
        }
    connections allowunconnected:
        mainIn --> app.mainIn;
        //app.mainOut --> mainOut;
		subIn --> app.subIn;
        //app.subOut --> subOut;
}
