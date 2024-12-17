import Experience from "@/components/Experience";

export default function Home() {
    return (
        <main className="bg-lime-300 w-screen h-screen">
            <Overlay />
            <Experience />
        </main>
    );
}

const Overlay = () => (
    <div className="absolute top-6 left-6 flex flex-col select-none text-white">
        <span className="font-semibold text-5xl">PanaBot</span>
        <span className="font-extralight"> version: 0.0.1</span>
    </div>
)
