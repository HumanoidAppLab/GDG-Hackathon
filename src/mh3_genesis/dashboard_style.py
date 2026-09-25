"""Provide the readable stylesheet bundled with the browser dashboard."""

STYLE = """
:root
{
    --bg: #0b1118;
    --panel: #151e29;
    --line: #29384a;
    --text: #e9f0f7;
    --muted: #8fa2b7;
    --accent: #48b6ff;
    --danger: #ff5c6c;
}
*
{
    box-sizing: border-box;
}
body
{
    margin: 0;
    background: var(--bg);
    color: var(--text);
    font: 14px system-ui, sans-serif;
}
header
{
    position: sticky;
    top: 0;
    z-index: 2;
    display: flex;
    align-items: center;
    gap: 16px;
    padding: 14px 20px;
    background: #0f1721;
    border-bottom: 1px solid var(--line);
}
h1
{
    font-size: 18px;
    margin: 0;
}
#status
{
    color: var(--muted);
}
main
{
    display: grid;
    grid-template-columns: 300px 1fr;
    gap: 16px;
    padding: 16px;
}
.card
{
    background: var(--panel);
    border: 1px solid var(--line);
    border-radius: 12px;
    padding: 16px;
}
h2
{
    font-size: 15px;
    margin: 0 0 14px;
}
.workspace
{
    display: grid;
    gap: 16px;
    min-width: 0;
}
.camera-row
{
    margin-top: 16px;
}
.camera-row h3
{
    font-size: 13px;
    margin: 0 0 10px;
}
.camera-grid
{
    display: grid;
    grid-template-columns: repeat(3, minmax(0, 1fr));
    gap: 12px;
}
.camera-grid figure
{
    margin: 0;
    min-width: 0;
}
.camera-grid img
{
    display: block;
    width: 100%;
    aspect-ratio: 16 / 9;
    object-fit: contain;
    background: #0e1620;
    border-radius: 6px;
}
.camera-grid figcaption
{
    margin-top: 6px;
    color: var(--muted);
    font-size: 12px;
}
.drive-grid
{
    display: grid;
    grid-template-columns: repeat(3, 64px);
    gap: 8px;
    justify-content: center;
    margin: 14px 0;
}
button
{
    border: 1px solid #3b526a;
    background: #223044;
    color: var(--text);
    border-radius: 8px;
    padding: 11px;
    cursor: pointer;
}
button:hover
{
    background: #2b4057;
}
button.stop
{
    background: #4a2028;
    border-color: #883945;
}
button.home
{
    width: 100%;
    margin-top: 8px;
}
.speed label
{
    display: grid;
    grid-template-columns: 90px 1fr 48px;
    align-items: center;
    gap: 8px;
    margin: 8px 0;
    color: var(--muted);
}
#joint-groups
{
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(390px, 1fr));
    gap: 16px;
}
.joint-row
{
    display: grid;
    grid-template-columns: 170px 1fr 72px;
    gap: 10px;
    align-items: center;
    margin: 8px 0;
}
.joint-row span
{
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
    color: #c8d5e3;
}
input[type=range]
{
    width: 100%;
    accent-color: var(--accent);
}
input[type=number]
{
    width: 72px;
    background: #0e1620;
    color: var(--text);
    border: 1px solid var(--line);
    border-radius: 6px;
    padding: 5px;
}
.hint
{
    color: var(--muted);
    font-size: 12px;
    line-height: 1.5;
}
@media (max-width: 1000px)
{
    .camera-grid
    {
        grid-template-columns: 1fr;
    }
}
@media (max-width: 800px)
{
    main
    {
        grid-template-columns: 1fr;
    }
    #joint-groups
    {
        grid-template-columns: 1fr;
    }
}
"""
