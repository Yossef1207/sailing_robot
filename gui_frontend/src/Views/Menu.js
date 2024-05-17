import React, { Component } from "react";
//Import Menu-Elements
import { Menubar } from "primereact/menubar";
import { ProgressSpinner } from 'primereact/progressspinner';
import { ToggleButton } from "primereact/togglebutton";
import { Button } from "primereact/button";
import { Dialog } from "primereact/dialog";
import { Dropdown } from "primereact/dropdown";
import { Toast } from "primereact/toast";
import Cookies from "js-cookie";
import "../styles/menu.css";

export class Menu extends Component {
    constructor(props) {
        super(props);
        this.state = {
            loading: false,
            items: [
                    {
                        label: "Home",
                        icon: "pi pi-fs pi-home",
                        command: (e) => {
                            window.location.href = "/";
                        },
                    }
            ],
            itemsEnd: []
        };
    }

    //----------------General page functions------------------

    render() {
        const start = (
            <img
                alt="logo"
                src="./media/logo.png"
                height="40"
                className="mr-2"
            />
        );
        if(this.state.loading == false) {
            return (
                <span>
                    <Toast ref={(el) => (this.toast = el)} position="tocenter" />
                    <Menubar
                        model={this.state.items}
                        start={start}
                    />
                </span>
            );
        } else {
            return (
                <span>
                    <ProgressSpinner />
                </span>
            );
        }
    }
}

export default Menu;
