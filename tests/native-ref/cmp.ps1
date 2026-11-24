# Compares two trace dumps numerically: cmp.ps1 <fileA> <fileB> [relTol]
# Lines: "step N" / "<tag> pos=(...) q=(...) lv=(...) av=(...)"
param([string]$a, [string]$b, [double]$tol = 1e-6)

function Parse-nums([string]$line) {
    $nums = @()
    foreach ($m in [regex]::Matches($line, '[-+0-9.eE]+')) {
        $v = 0.0
        if ([double]::TryParse($m.Value, [System.Globalization.NumberStyles]::Float,
                [System.Globalization.CultureInfo]::InvariantCulture, [ref]$v)) { $nums += $v }
    }
    return $nums
}

$la = Get-Content $a
$lb = Get-Content $b
if ($la.Count -ne $lb.Count) { Write-Host "line count differs: $($la.Count) vs $($lb.Count)"; }
$n = [Math]::Min($la.Count, $lb.Count)
$worst = 0.0; $worstLine = -1
for ($i = 0; $i -lt $n; $i++) {
    if ($la[$i] -ne $lb[$i]) {
        $na = Parse-nums $la[$i]; $nb = Parse-nums $lb[$i]
        if ($na.Count -ne $nb.Count) { Write-Host "line $($i+1): field mismatch"; Write-Host "  A: $($la[$i])"; Write-Host "  B: $($lb[$i])"; exit 1 }
        $diff = 0.0
        for ($j = 0; $j -lt $na.Count; $j++) { $diff = [Math]::Max($diff, [Math]::Abs($na[$j] - $nb[$j])) }
        if ($diff -gt $worst) { $worst = $diff; $worstLine = $i + 1 }
        if ($diff -gt $tol) {
            Write-Host "line $($i+1) maxabs diff = $diff"
            Write-Host "  A: $($la[$i])"
            Write-Host "  B: $($lb[$i])"
            exit 1
        }
    }
}
Write-Host "OK (max abs diff $worst at line $worstLine)"
