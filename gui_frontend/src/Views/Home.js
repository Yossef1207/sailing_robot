import React, { Component } from "react"
import L from 'leaflet'
import { MapContainer, TileLayer, Marker, Polyline } from 'react-leaflet'
import 'leaflet/dist/leaflet.css'
import "leaflet/dist/images/marker-icon.png"
import icon from 'leaflet/dist/images/marker-icon.png'
import iconShadow from 'leaflet/dist/images/marker-shadow.png'
import config from "../config.json"
//Import SocketIO-Components
import socketIOClient from "socket.io-client"

let DefaultIcon = L.icon({
    iconUrl: icon,
    shadowUrl: iconShadow
})

L.Marker.prototype.options.icon = DefaultIcon

const polyline = [
    [53.456240, 10.012865],
    [53.455448, 10.014560],
    [53.454733, 10.017414],
    [53.456585, 10.021298],
    [53.456470, 10.023229],
    [53.456790, 10.024259],
    [53.459000, 10.023594],
    [53.460392, 10.023465],
    [53.461427, 10.023036],
    [53.461747, 10.020847],
    [53.460993, 10.020139],
    [53.460175, 10.020719],
    [53.459537, 10.019946],
    [53.459294, 10.018938],
    [53.458872, 10.018959],
    [53.458732, 10.017457],
    [53.458808, 10.015998],
    [53.458527, 10.014882],
    [53.457850, 10.013841],
    [53.457032, 10.013541],
    [53.456240, 10.012865]
  ]

const blackOptions = { color: 'black' }
const purpleOptions = { color: 'purple' }
const redOptions = { color: 'red' }

export class Home extends Component {
    constructor(props) {
        super(props);
        this.state = {
            targets: []
        };
    }

    componentDidMount() {
        const socket = socketIOClient(config.BACKEND_URL);
        //Connect to Backend as frontend
        socket.emit("log_on", "frontend");
        socket.on("sailboat_data", (msg) => {
            console.log(msg);
            this.setState({targets: msg})
            //Client.start(socket)
        });

    }

    render() {
        return(
                    <MapContainer center={[53.4567, 10.0160]} zoom={15} scrollWheelZoom={true}>
    <TileLayer
      attribution='&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
      url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
      style={{ height: '10vh', width: '10wh' }}
    />
    <Marker 
    position={polyline[0]} >
    </Marker>
    <Marker 
    position={polyline[1]} >
    </Marker>
    <Marker 
    position={polyline[2]}>
    </Marker>
    <Marker 
    position={polyline[3]}>
    </Marker>
    <Marker 
    position={polyline[4]}>
    </Marker>
    <Marker 
    position={polyline[5]}>
    </Marker>
    <Marker 
    position={polyline[6]}>
    </Marker>
    <Marker 
    position={polyline[7]}>
    </Marker>
    <Marker 
    position={polyline[8]}>
    </Marker>
    <Marker 
    position={polyline[9]}>
    </Marker>
    <Marker 
    position={polyline[10]}>
    </Marker>
    <Marker 
    position={polyline[11]}>
    </Marker>
    <Marker 
    position={polyline[12]}>
    </Marker>
    <Marker 
    position={polyline[13]}>
    </Marker>
    <Marker 
    position={polyline[14]}>
    </Marker>
    <Marker 
    position={polyline[15]}>
    </Marker>
    <Marker 
    position={polyline[16]}>
    </Marker>
    <Marker 
    position={polyline[17]}>
    </Marker>
    <Marker 
    position={polyline[18]}>
    </Marker>
    <Marker 
    position={polyline[19]}>
    </Marker>
    <Marker 
    position={polyline[20]}>
    </Marker>
    <Polyline pathOptions={redOptions} positions={polyline} />
  </MapContainer>
            );
        }
    }

export default Home;
