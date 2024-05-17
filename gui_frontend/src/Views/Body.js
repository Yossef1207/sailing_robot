import React, { Component } from "react";
import { Container } from 'reactstrap'
import { Menu } from './Menu' 
import config from '../config.json'
import "../styles/body.css"

export class Body extends Component {
    static displayName = Body.name
    constructor(props) {
        super(props);
        this.state = {  
            author: config.Authors
        }
    }

    render() { 
        return (  
            <div>
                <Menu />
                <Container>{this.props.children} </Container>
                <footer class='footer'>
                    <p>Author: {this.state.author} </p>
                </footer>
            </div>
        );
    }
}