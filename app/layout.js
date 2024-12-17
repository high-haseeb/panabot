import "./globals.css";

export const metadata = {
    title: "PanaBot",
    description: "simulation of a robotic helper",
};

export default function RootLayout({ children }) {
    return (
        <html lang="en">
            <body
                className={`antialiased`}
            >
                {children}
            </body>
        </html>
    );
}
