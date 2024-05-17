import './App.css';
//Style-Imports of Prime-React
import 'primereact/resources/primereact.min.css'
import 'primeicons/primeicons.css'
import 'primereact/resources/themes/saga-blue/theme.css'
import React, {Component} from 'react'

//import { Switch } from 'react-router-dom' -- For multiple /create/...
import {
  BrowserRouter,
  Routes,
  Route
} from "react-router-dom";

//import single components
import { Home } from './Views/Home'
import { Body } from './Views/Body'

export default class App extends Component {
  static displayName = App.name

  constructor(props) {
    super(props)
    this.state = {
      curAcc: undefined
    }
  }
  
  componentDidMount() {
  }

  render() {
    return(      
        <BrowserRouter>
            <Body>
              <Routes>
                <Route exact path='/' element={<Home />} />
              </Routes>
            </Body>
        </BrowserRouter>
        
    );
  }
}
